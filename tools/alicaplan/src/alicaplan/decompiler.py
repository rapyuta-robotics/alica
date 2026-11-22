import os
from collections import Counter

from .jsonio import INT_MAX
from .source import (
    ANNOTATED_DEFAULTS,
    BEHAVIOUR_DEFAULTS,
    COND_DEFAULTS,
    DEFAULT_ACCESS,
    EP_DEFAULTS,
    LAYOUT_SUFFIX,
    MODULE_SUFFIX,
    PLAN_DEFAULTS,
    REPO_DEFAULT_NAME,
    ROLESET_DEFAULTS,
    SYNC_DEFAULTS,
    TERMINAL_KEY,
    TRANSITION_DEFAULTS,
    TRANSITION_SEP,
    WRAPPER_DEFAULTS,
    annotated_default_name,
    dup_keys,
    same,
    terminal_default_name,
)

ABSTRACT_KINDS = ("plan", "behaviour", "plantype", "placeholder")


class DecompileError(Exception):
    pass


class Decompiler:
    def __init__(self, docs, keep_ids=True):
        self.docs = docs
        self.keep_ids = keep_ids
        self.top = {}
        self.names = {}
        self.tasks = {}
        self.task_names = Counter()
        self.conditions = {}
        self.condition_names = Counter()
        for doc in docs:
            data = doc.data
            self.top[data["id"]] = doc
            self.names.setdefault(data["name"], []).append(doc)
            if doc.kind == "taskrepository":
                for task in data["tasks"]:
                    self.tasks[task["id"]] = (doc, task)
                    self.task_names[task["name"]] += 1
            elif doc.kind == "conditionrepository":
                for cond in data["conditions"]:
                    self.conditions[cond["id"]] = (doc, cond)
                    self.condition_names[cond["name"]] += 1

    def put_id(self, out, obj):
        if self.keep_ids:
            out["id"] = obj["id"]

    @staticmethod
    def split(ref):
        if not isinstance(ref, str) or "#" not in ref:
            return None, None
        file_part, _, id_part = ref.rpartition("#")
        try:
            return file_part, int(id_part)
        except ValueError:
            return None, None

    def ref_top(self, ref, kinds):
        file_part, elem_id = self.split(ref)
        doc = self.top.get(elem_id)
        if doc is None or doc.kind not in kinds or file_part != doc.basename or doc.stem != doc.data["name"]:
            return ref
        rivals = [d for d in self.names.get(doc.stem, []) if d.kind in kinds]
        return doc.stem if len(rivals) == 1 else doc.basename

    def ref_nested(self, ref, table, counts):
        file_part, elem_id = self.split(ref)
        found = table.get(elem_id)
        if found is None:
            return ref
        repo, item = found
        name = item["name"]
        if file_part != repo.basename or repo.stem != repo.data["name"] or not name or "/" in name or "#" in name:
            return ref
        return name if counts[name] == 1 else f"{repo.stem}/{name}"

    def ref_task(self, ref):
        return self.ref_nested(ref, self.tasks, self.task_names)

    def ref_condition(self, ref):
        if ref is None:
            return None
        return self.ref_nested(ref, self.conditions, self.condition_names)

    def defaults(self, out, obj, table, renames=None):
        renames = renames or {}
        for yaml_key, default in table.items():
            json_key = renames.get(yaml_key, yaml_key)
            if json_key in obj and not same(obj[json_key], default):
                out[yaml_key] = obj[json_key]

    def blackboard(self, entries, owner_kind):
        result = {}
        for entry in entries:
            key = entry["key"]
            if key in result:
                raise DecompileError(f"blackboard key '{key}' is defined twice")
            item = {"type": entry["type"]}
            if entry.get("access") != DEFAULT_ACCESS[owner_kind]:
                item["access"] = entry.get("access")
            if entry.get("defaultValue") is not None:
                item["default"] = entry["defaultValue"]
            if entry.get("comment") != "":
                item["comment"] = entry.get("comment")
            self.put_id(item, entry)
            result[key] = item["type"] if list(item) == ["type"] else item
        return result

    def condition_object(self, cond, scopes=None):
        if cond is None:
            return None
        out = {}
        self.defaults(out, cond, COND_DEFAULTS)
        if out.get("quantifiers"):
            quantifiers = []
            for quant in out["quantifiers"]:
                if isinstance(quant, dict):
                    quant = dict(quant)
                    if scopes and quant.get("scope") in scopes:
                        quant["scope"] = scopes[quant["scope"]]
                    if not self.keep_ids:
                        quant.pop("id", None)
                quantifiers.append(quant)
            out["quantifiers"] = quantifiers
        self.put_id(out, cond)
        return out

    def bindings(self, bindings):
        return [
            dict(b, subPlan=self.ref_top(b["subPlan"], ABSTRACT_KINDS)) if isinstance(b, dict) and isinstance(b.get("subPlan"), str) else b
            for b in bindings
        ]

    def key_mapping(self, out, mapping, where):
        if not mapping:
            return
        inputs = {}
        for item in mapping.get("input") or []:
            child = item["childKey"]
            if child in inputs:
                raise DecompileError(f"{where}: child key '{child}' has two inputs")
            if item.get("parentKey") is not None:
                inputs[child] = item["parentKey"]
            else:
                inputs[child] = {"value": item.get("value")}
        outputs = {}
        for item in mapping.get("output") or []:
            parent = item["parentKey"]
            if parent in outputs:
                raise DecompileError(f"{where}: parent key '{parent}' has two outputs")
            outputs[parent] = item["childKey"]
        if inputs:
            out["in"] = inputs
        if outputs:
            out["out"] = outputs

    def module_library(self, doc):
        data = doc.data
        if doc.kind in ("plan", "behaviour"):
            return data.get("libraryName", "")
        if doc.kind == "conditionrepository":
            libs = Counter(c.get("libraryName", "") for c in data["conditions"])
            return libs.most_common(1)[0][0] if libs else ""
        return ""

    def decompile(self):
        modules = {}
        taken = Counter(os.path.join(os.path.dirname(d.relpath), d.stem) for d in self.docs)
        for doc in self.docs:
            base = os.path.join(os.path.dirname(doc.relpath), doc.stem)
            if taken[base] > 1:
                base = base + "." + doc.ext[1:]
            library = self.module_library(doc)
            module = {}
            if library:
                module["library"] = library
            layout = {}
            body = getattr(self, "doc_" + doc.kind)(doc, library, layout)
            section = {
                "plan": "plans",
                "behaviour": "behaviours",
                "plantype": "plantypes",
                "placeholder": "placeholders",
                "taskrepository": "task_repository",
                "conditionrepository": "condition_repository",
                "roleset": "roleset",
            }[doc.kind]
            if section in ("task_repository", "condition_repository", "roleset"):
                module[section] = body
            else:
                module[section] = {doc.stem: body}
            modules[base + MODULE_SUFFIX] = module
            if layout:
                modules[base + LAYOUT_SUFFIX] = {doc.stem: layout}
        return modules

    def top_common(self, doc, out, library):
        data = doc.data
        if data["name"] != doc.stem:
            raise DecompileError(f"{doc.relpath}: element name '{data['name']}' differs from the file name")
        self.put_id(out, data)
        if data.get("libraryName", "") != library:
            out["library"] = data.get("libraryName", "")

    def doc_behaviour(self, doc, library, layout):
        data = doc.data
        out = {}
        self.top_common(doc, out, library)
        renames = {"implementation": "implementationName", "pre": "preCondition", "runtime": "runtimeCondition", "post": "postCondition"}
        table = {k: v for k, v in BEHAVIOUR_DEFAULTS.items() if k not in ("pre", "runtime", "post")}
        self.defaults(out, data, table, renames)
        for key in ("pre", "runtime", "post"):
            cond = data.get(renames[key])
            if cond is not None:
                out[key] = self.condition_object(cond)
        if data.get("blackboard"):
            out["blackboard"] = self.blackboard(data["blackboard"], "behaviour")
        return out

    def doc_placeholder(self, doc, library, layout):
        data = doc.data
        out = {}
        self.top_common(doc, out, library)
        if data.get("comment"):
            out["comment"] = data["comment"]
        if data.get("blackboard"):
            out["blackboard"] = self.blackboard(data["blackboard"], "placeholder")
        return out

    def doc_plantype(self, doc, library, layout):
        data = doc.data
        out = {}
        self.top_common(doc, out, library)
        if data.get("comment"):
            out["comment"] = data["comment"]
        plans = []
        for ap in data["annotatedPlans"]:
            ref = self.ref_top(ap["plan"], ("plan",))
            target = self.top.get(self.split(ap["plan"])[1]) if "#" not in ref else None
            opts = {}
            self.defaults(opts, ap, ANNOTATED_DEFAULTS)
            if ap.get("name") != annotated_default_name(target.stem if target else None):
                opts["name"] = ap.get("name")
            self.put_id(opts, ap)
            plans.append({ref: opts} if opts else ref)
        out["plans"] = plans
        if data.get("variables"):
            out["variables"] = data["variables"]
        if data.get("variableBindings"):
            out["variableBindings"] = self.bindings(data["variableBindings"])
        return out

    def repo_common(self, doc, out):
        data = doc.data
        if data["name"] != doc.stem:
            raise DecompileError(f"{doc.relpath}: element name '{data['name']}' differs from the file name")
        if data["name"] != REPO_DEFAULT_NAME[doc.kind]:
            out["name"] = data["name"]
        self.put_id(out, data)
        if data.get("comment"):
            out["comment"] = data["comment"]

    def doc_taskrepository(self, doc, library, layout):
        out = {}
        self.repo_common(doc, out)
        tasks = []
        for task in doc.data["tasks"]:
            opts = {}
            self.put_id(opts, task)
            if task.get("comment"):
                opts["comment"] = task["comment"]
            tasks.append({task["name"]: opts} if opts else task["name"])
        out["tasks"] = tasks
        return out

    def doc_conditionrepository(self, doc, library, layout):
        out = {}
        self.repo_common(doc, out)
        conditions = {}
        for cond in doc.data["conditions"]:
            opts = {}
            self.put_id(opts, cond)
            if cond.get("libraryName", "") != library:
                opts["library"] = cond.get("libraryName", "")
            if cond.get("comment"):
                opts["comment"] = cond["comment"]
            if cond.get("blackboard"):
                opts["blackboard"] = self.blackboard(cond["blackboard"], "condition")
            conditions[cond["name"]] = opts or None
        out["conditions"] = conditions
        return out

    def doc_roleset(self, doc, library, layout):
        data = doc.data
        out = {}
        self.repo_common(doc, out)
        self.defaults(out, data, {k: v for k, v in ROLESET_DEFAULTS.items() if k != "comment"})
        roles = {}
        for role in data["roles"]:
            opts = {}
            self.put_id(opts, role)
            if role.get("comment"):
                opts["comment"] = role["comment"]
            if role.get("characteristics"):
                opts["characteristics"] = role["characteristics"]
            priorities = {}
            for ref, value in (role.get("taskPriorities") or {}).items():
                priorities[self.ref_task(ref)] = value
            if priorities:
                opts["priorities"] = priorities
            roles[role["name"]] = opts or None
        out["roles"] = roles
        return out

    def state_keys(self, states):
        names = Counter(s.get("name") for s in states)
        keys = {}
        used = set()
        for state in states:
            name = state.get("name")
            terminal = state.get("type") == "TerminalState"
            key = name
            valid = (
                isinstance(name, str)
                and name
                and names[name] == 1
                and TRANSITION_SEP not in name
                and not name.startswith("#")
                and not TERMINAL_KEY.match(name)
            )
            if terminal and isinstance(name, str):
                kind = "success" if state.get("success") else "failure"
                for candidate in [kind] + [f"{kind}.{n}" for n in range(2, 100)]:
                    if terminal_default_name(candidate) == name and names[name] == 1:
                        key = candidate
                        valid = True
                        break
            if not valid or key in used:
                key = f"#{state['id']}"
            used.add(key)
            keys[state["id"]] = key
        return keys

    def doc_plan(self, doc, library, layout):
        data = doc.data
        out = {}
        self.top_common(doc, out, library)
        renames = {"implementation": "implementationName", "master": "masterPlan", "pre": "preCondition", "runtime": "runtimeCondition"}
        table = {k: v for k, v in PLAN_DEFAULTS.items() if k not in ("pre", "runtime")}
        self.defaults(out, data, table, renames)
        skeys = self.state_keys(data["states"])
        where = doc.relpath
        ep_refs = [self.ref_task(ep["task"]) for ep in data["entryPoints"]]
        scopes = {data["id"]: "plan"}
        scopes.update({sid: f"state:{key}" for sid, key in skeys.items()})
        for ep, ep_key in zip(data["entryPoints"], dup_keys([str(r) for r in ep_refs], " #")):
            scopes[ep["id"]] = f"ep:{ep_key}"
        for key in ("pre", "runtime"):
            cond = data.get(renames[key])
            if cond is not None:
                out[key] = self.condition_object(cond, scopes)
        if data.get("blackboard"):
            out["blackboard"] = self.blackboard(data["blackboard"], "plan")

        ep_layout = {}
        eps = []
        ep_task_refs = [self.ref_task(ep["task"]) for ep in data["entryPoints"]]
        for ep, ep_key, task_ref in zip(data["entryPoints"], dup_keys([str(r) for r in ep_task_refs], " #"), ep_task_refs):
            if ep.get("plan") != data["id"]:
                raise DecompileError(f"{where}: entry point {ep['id']} belongs to plan {ep.get('plan')}")
            if ep["state"] not in skeys:
                raise DecompileError(f"{where}: entry point {ep['id']} starts in unknown state {ep['state']}")
            item = {"task": task_ref, "state": skeys[ep["state"]]}
            table = dict(EP_DEFAULTS)
            self.defaults(item, ep, table, {"min": "minCardinality", "max": "maxCardinality"})
            if item.get("max") == INT_MAX:
                del item["max"]
            self.put_id(item, ep)
            eps.append(item)
            ep_layout[ep_key] = [ep["positionWeb"]["x"], ep["positionWeb"]["y"]]
        out["entryPoints"] = eps

        ep_of_state = {ep["state"]: ep["id"] for ep in reversed(data["entryPoints"])}
        trans_of = {t["id"]: t for t in data["transitions"]}
        states = {}
        state_layout = {}
        for state in data["states"]:
            sid = state["id"]
            key = skeys[sid]
            swhere = f"{where}: state {key}"
            if state.get("parentPlan") != data["id"]:
                raise DecompileError(f"{swhere}: parentPlan is {state.get('parentPlan')}")
            if state.get("entryPoint") != ep_of_state.get(sid):
                raise DecompileError(f"{swhere}: entryPoint {state.get('entryPoint')} does not match the entry points")
            expect_out = [t["id"] for t in data["transitions"] if t["inState"] == sid]
            expect_in = [t["id"] for t in data["transitions"] if t["outState"] == sid]
            if state.get("outTransitions") != expect_out or state.get("inTransitions") != expect_in:
                raise DecompileError(f"{swhere}: inTransitions/outTransitions do not match the transitions")
            terminal = state.get("type") == "TerminalState"
            body = {}
            default_name = key
            keyword = TERMINAL_KEY.match(key)
            if keyword:
                default_name = terminal_default_name(key)
            elif key.startswith("#"):
                default_name = None
            if terminal and not keyword:
                body["terminal"] = "success" if state.get("success") else "failure"
            if state.get("name") != default_name:
                body["name"] = state.get("name")
            self.put_id(body, state)
            if state.get("comment"):
                body["comment"] = state["comment"]
            children = []
            refs = [self.ref_top(w["abstractPlan"], ABSTRACT_KINDS) for w in state["confAbstractPlanWrappers"]]
            for wrapper, ref in zip(state["confAbstractPlanWrappers"], refs):
                opts = {}
                self.defaults(opts, wrapper, WRAPPER_DEFAULTS)
                self.put_id(opts, wrapper)
                self.key_mapping(opts, wrapper.get("keyMapping"), swhere)
                children.append({ref: opts} if opts else ref)
            if terminal:
                if state.get("postCondition") is not None:
                    body["post"] = self.condition_object(state["postCondition"], scopes)
            if state.get("variableBindings"):
                body["variableBindings"] = self.bindings(state["variableBindings"])
            if children:
                if body:
                    body["children"] = children
                else:
                    body = children
            states[key] = body or None
            state_layout[key] = [state["positionWeb"]["x"], state["positionWeb"]["y"]]
        out["states"] = states

        labels = [f"{skeys[t['inState']]}{TRANSITION_SEP}{skeys[t['outState']]}" for t in data["transitions"]]
        tkeys = dup_keys(labels, " #")
        tkey_of = {t["id"]: k for t, k in zip(data["transitions"], tkeys)}
        transitions = []
        trans_layout = {}
        for tr, label, tkey in zip(data["transitions"], labels, tkeys):
            body = {}
            cond = self.ref_condition(tr.get("condition"))
            body["condition"] = cond
            self.defaults(body, tr, TRANSITION_DEFAULTS)
            self.put_id(body, tr)
            self.key_mapping(body, tr.get("keyMapping"), f"{where}: transition {label}")
            pre = tr.get("preCondition")
            if pre is None:
                body["pre"] = None
            else:
                pre_out = self.condition_object(pre, scopes)
                if pre_out:
                    body["pre"] = pre_out
            if list(body) == ["condition"] and cond is not None:
                transitions.append({label: cond})
            else:
                transitions.append({label: body})
            if tr.get("pointsWeb"):
                trans_layout[tkey] = [[p["x"], p["y"]] for p in tr["pointsWeb"]]
        out["transitions"] = transitions

        syncs = {}
        sync_layout = {}
        sync_names = Counter(s.get("name") for s in data["synchronisations"])
        for sync in data["synchronisations"]:
            name = sync.get("name")
            key = name if isinstance(name, str) and name and sync_names[name] == 1 and not name.startswith("#") else f"#{sync['id']}"
            body = {}
            if name != key:
                body["name"] = name
            self.put_id(body, sync)
            self.defaults(body, sync, SYNC_DEFAULTS)
            synced = []
            for tid in sync["syncedTransitions"]:
                tr = trans_of.get(tid)
                if tr is None or tr.get("synchronisation") != sync["id"]:
                    raise DecompileError(f"{where}: synchronisation {key} and transition {tid} disagree")
                synced.append(tkey_of[tid])
            body["transitions"] = synced
            syncs[key] = body
            sync_layout[key] = [sync["positionWeb"]["x"], sync["positionWeb"]["y"]]
        for tr in data["transitions"]:
            if tr.get("synchronisation") is not None and tr["id"] not in [t for s in data["synchronisations"] for t in s["syncedTransitions"]]:
                raise DecompileError(f"{where}: transition {tr['id']} names a synchronisation that does not list it")
        if syncs:
            out["synchronisations"] = syncs

        layout["entryPoints"] = ep_layout
        layout["states"] = state_layout
        if sync_layout:
            layout["synchronisations"] = sync_layout
        if trans_layout:
            layout["transitions"] = trans_layout
        return out


def decompile_docs(docs, keep_ids=True):
    bad = [d.relpath for d in docs if d.errors]
    if bad:
        raise DecompileError("cannot decompile files that do not parse: " + ", ".join(bad))
    return Decompiler(docs, keep_ids).decompile()
