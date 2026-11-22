import math
import os
from dataclasses import dataclass, field

from .jsonio import INT_MAX, Doc
from .source import (
    ANNOTATED_DEFAULTS,
    BB_DEFAULTS,
    BEHAVIOUR_DEFAULTS,
    COND_DEFAULTS,
    DEFAULT_ACCESS,
    EP_DEFAULTS,
    EXT_KIND,
    KIND_EXT,
    MODULE_KEYS,
    PLAN_DEFAULTS,
    REPO_DEFAULT_NAME,
    REPO_SECTION_KIND,
    ROLE_DEFAULTS,
    ROLESET_DEFAULTS,
    SECTION_KIND,
    SYNC_DEFAULTS,
    TERMINAL_KEY,
    TRANSITION_DEFAULTS,
    TRANSITION_SEP,
    WRAPPER_DEFAULTS,
    annotated_default_name,
    dup_keys,
    hash_id,
    layout_path,
    load_yaml,
    source_files,
    terminal_default_name,
)

ABSTRACT_KINDS = ("plan", "behaviour", "plantype", "placeholder")


class CompileError(Exception):
    def __init__(self, messages):
        super().__init__("\n".join(messages))
        self.messages = messages


@dataclass
class Module:
    relpath: str
    data: dict
    layout: dict

    @property
    def reldir(self):
        return os.path.dirname(self.relpath)


@dataclass
class Element:
    kind: str
    name: str
    module: Module
    spec: dict
    where: str
    id: int = 0
    nested: dict = field(default_factory=dict)

    @property
    def ext(self):
        return KIND_EXT[self.kind]

    @property
    def filename(self):
        return self.name + self.ext

    @property
    def outpath(self):
        return os.path.join(self.module.reldir, self.filename).replace(os.sep, "/")

    @property
    def library(self):
        return self.spec.get("library", self.module.data.get("library", ""))


@dataclass
class Nested:
    name: str
    repo: Element
    spec: dict
    id: int


class Compiler:
    def __init__(self, modules):
        self.modules = modules
        self.errors = []
        self.elements = []
        self.by_name = {}
        self.tasks = {}
        self.conditions = {}

    def err(self, where, message):
        self.errors.append(f"{where}: {message}")

    def compile(self):
        for module in self.modules:
            self.collect(module)
        for el in self.elements:
            self.assign_nested(el)
        docs = []
        for el in self.elements:
            data = getattr(self, "build_" + el.kind)(el)
            if data is not None:
                docs.append(Doc(el.outpath, data))
        if self.errors:
            raise CompileError(self.errors)
        return docs

    def check_keys(self, where, spec, allowed):
        if not isinstance(spec, dict):
            self.err(where, f"expected a mapping, got {type(spec).__name__}")
            return False
        for key in spec:
            if key not in allowed:
                self.err(where, f"unknown key '{key}' (allowed: {', '.join(sorted(allowed))})")
        return True

    def ident(self, spec, qualified):
        explicit = spec.get("id") if isinstance(spec, dict) else None
        if explicit is not None:
            if not isinstance(explicit, int) or isinstance(explicit, bool):
                self.err(qualified, f"id must be an integer, got {explicit!r}")
                return hash_id(qualified)
            return explicit
        return hash_id(qualified)

    def collect(self, module):
        where = module.relpath
        data = module.data
        if data is None:
            return
        if not self.check_keys(where, data, MODULE_KEYS):
            return
        for section, kind in SECTION_KIND.items():
            entries = data.get(section) or {}
            if not isinstance(entries, dict):
                self.err(f"{where}: {section}", "expected a mapping of name to definition")
                continue
            for name, spec in entries.items():
                self.add_element(kind, name, module, {} if spec is None else spec, f"{where}: {section}.{name}")
        for section, kind in REPO_SECTION_KIND.items():
            spec = data.get(section)
            if spec is None:
                continue
            if not isinstance(spec, dict):
                self.err(f"{where}: {section}", "expected a mapping")
                continue
            name = spec.get("name", REPO_DEFAULT_NAME[kind])
            self.add_element(kind, name, module, spec, f"{where}: {section}")

    def add_element(self, kind, name, module, spec, where):
        if not isinstance(name, str) or not name or "#" in name or "/" in name:
            self.err(where, f"invalid name {name!r}")
            return
        el = Element(kind, name, module, spec, where)
        key = (name, kind)
        if key in self.by_name:
            self.err(where, f"{kind} '{name}' is already defined in {self.by_name[key].where}")
            return
        self.by_name[key] = el
        el.id = self.ident(spec, f"{el.ext[1:]}:{name}")
        self.elements.append(el)

    def assign_nested(self, el):
        if el.kind == "taskrepository":
            tasks = el.spec.get("tasks") or []
            if not isinstance(tasks, list):
                self.err(el.where, "tasks must be a list")
                return
            for item in tasks:
                name, spec = self.single(item, el.where + ".tasks")
                if name is None:
                    continue
                self.check_keys(f"{el.where}.tasks.{name}", spec, {"id", "comment"})
                if name in el.nested:
                    self.err(el.where, f"task '{name}' is listed twice")
                    continue
                nested = Nested(name, el, spec, self.ident(spec, f"tsk:{el.name}/task:{name}"))
                el.nested[name] = nested
                self.tasks.setdefault(name, []).append(nested)
        elif el.kind == "conditionrepository":
            conds = el.spec.get("conditions") or {}
            if not isinstance(conds, dict):
                self.err(el.where, "conditions must be a mapping of name to definition")
                return
            for name, spec in conds.items():
                spec = {} if spec is None else spec
                if not isinstance(name, str) or not name or "#" in name or "/" in name:
                    self.err(el.where, f"invalid condition name {name!r}")
                    continue
                self.check_keys(f"{el.where}.conditions.{name}", spec, {"id", "comment", "library", "blackboard"})
                nested = Nested(name, el, spec, self.ident(spec, f"cnd:{el.name}/condition:{name}"))
                el.nested[name] = nested
                self.conditions.setdefault(name, []).append(nested)

    def single(self, item, where):
        if isinstance(item, str):
            return item, {}
        if isinstance(item, dict) and len(item) == 1:
            name, spec = next(iter(item.items()))
            if not isinstance(name, str):
                self.err(where, f"name must be a string, got {name!r}")
                return None, None
            return name, {} if spec is None else spec
        self.err(where, f"expected a name or a single-key mapping, got {item!r}")
        return None, None

    def ref_abstract(self, ref, where, kinds=ABSTRACT_KINDS):
        if not isinstance(ref, str) or not ref:
            self.err(where, f"invalid reference {ref!r}")
            return None, None
        if "#" in ref:
            return ref, None
        stem, ext = os.path.splitext(ref)
        if ext in EXT_KIND and EXT_KIND[ext] in kinds:
            el = self.by_name.get((stem, EXT_KIND[ext]))
            if el is None:
                self.err(where, f"no {EXT_KIND[ext]} named '{stem}'")
                return None, None
        else:
            found = [self.by_name[(ref, k)] for k in kinds if (ref, k) in self.by_name]
            if not found:
                self.err(where, f"no {' or '.join(kinds)} named '{ref}'")
                return None, None
            if len(found) > 1:
                options = ", ".join(e.filename for e in found)
                self.err(where, f"'{ref}' is ambiguous; write one of {options}")
                return None, None
            el = found[0]
        return f"{el.filename}#{el.id}", el

    def ref_nested(self, ref, where, table, what):
        if not isinstance(ref, str) or not ref:
            self.err(where, f"invalid {what} reference {ref!r}")
            return None, None
        if "#" in ref:
            return ref, None
        repo_name, _, name = ref.rpartition("/")
        found = [n for n in table.get(name, []) if not repo_name or n.repo.name == repo_name]
        if not found:
            self.err(where, f"no {what} named '{ref}'")
            return None, None
        if len(found) > 1:
            options = ", ".join(f"{n.repo.name}/{n.name}" for n in found)
            self.err(where, f"{what} '{ref}' is ambiguous; write one of {options}")
            return None, None
        nested = found[0]
        return f"{nested.repo.filename}#{nested.id}", nested

    def blackboard(self, spec, where, owner_kind, qualified):
        if spec is None:
            return []
        if not isinstance(spec, dict):
            self.err(where, "blackboard must be a mapping of key to type or definition")
            return []
        entries = []
        for key, item in spec.items():
            if not isinstance(key, str):
                self.err(where, f"blackboard key must be a string, got {key!r}")
                continue
            if isinstance(item, str):
                item = {"type": item}
            if not self.check_keys(f"{where}.blackboard.{key}", item, {"type", "access", "default", "comment", "id"}):
                continue
            if "type" not in item:
                self.err(f"{where}.blackboard.{key}", "type is required")
            entries.append(
                {
                    "access": item.get("access", DEFAULT_ACCESS[owner_kind]),
                    "comment": item.get("comment", BB_DEFAULTS["comment"]),
                    "defaultValue": item.get("default", BB_DEFAULTS["default"]),
                    "id": self.ident(item, f"{qualified}/bb:{key}"),
                    "key": key,
                    "type": item.get("type"),
                }
            )
        return entries

    def key_mapping(self, spec_in, spec_out, where):
        inputs, outputs = [], []
        if spec_in is not None:
            if not isinstance(spec_in, dict):
                self.err(where, "'in' must map child key to parent key or {value: ...}")
            else:
                for child, source in spec_in.items():
                    if isinstance(source, str):
                        inputs.append({"childKey": child, "parentKey": source, "value": None})
                    elif isinstance(source, dict) and set(source) == {"value"} and source["value"] is not None:
                        inputs.append({"childKey": child, "parentKey": None, "value": source["value"]})
                    else:
                        self.err(where, f"input to '{child}' must be a parent key or {{value: ...}}")
        if spec_out is not None:
            if not isinstance(spec_out, dict):
                self.err(where, "'out' must map parent key to child key")
            else:
                for parent, child in spec_out.items():
                    if not isinstance(child, str):
                        self.err(where, f"output into '{parent}' must name a child key")
                        continue
                    outputs.append({"childKey": child, "parentKey": parent})
        return {"input": inputs, "output": outputs}

    def condition_object(self, spec, where, qualified, scopes=None):
        if spec is None:
            return None
        if not self.check_keys(where, spec, set(COND_DEFAULTS) | {"id"}):
            return None
        data = {key: spec.get(key, default) for key, default in COND_DEFAULTS.items()}
        data["id"] = self.ident(spec, qualified)
        quantifiers = []
        for index, quant in enumerate(data["quantifiers"] or []):
            if not isinstance(quant, dict):
                self.err(where, f"quantifier must be a mapping, got {quant!r}")
                continue
            if not self.check_keys(where + ".quantifiers", quant, {"id", "name", "comment", "quantifierType", "scope", "sorts"}):
                continue
            scope = quant.get("scope")
            if isinstance(scope, str):
                if scopes is None or scope not in scopes:
                    self.err(where, f"quantifier scope {scope!r} must be 'plan', 'state:<name>' or 'ep:<task>' of this plan")
                else:
                    scope = scopes[scope]
            quantifiers.append(
                {
                    "comment": quant.get("comment", ""),
                    "id": self.ident(quant, f"{qualified}/quantifier:{index}"),
                    "name": quant.get("name", ""),
                    "quantifierType": quant.get("quantifierType", "all"),
                    "scope": scope,
                    "sorts": quant.get("sorts", []),
                }
            )
        if data["quantifiers"]:
            data["quantifiers"] = quantifiers
        return data

    def bindings(self, spec, where):
        result = []
        for binding in spec or []:
            if isinstance(binding, dict) and isinstance(binding.get("subPlan"), str) and "#" not in binding["subPlan"]:
                refstr, _ = self.ref_abstract(binding["subPlan"], where)
                binding = dict(binding, subPlan=refstr)
            result.append(binding)
        return result

    def build_behaviour(self, el):
        spec = el.spec
        allowed = set(BEHAVIOUR_DEFAULTS) | {"id", "library", "blackboard"}
        if not self.check_keys(el.where, spec, allowed):
            return None
        q = f"beh:{el.name}"
        get = lambda k: spec.get(k, BEHAVIOUR_DEFAULTS[k])  # noqa: E731
        return {
            "blackboard": self.blackboard(spec.get("blackboard"), el.where, "behaviour", q),
            "comment": get("comment"),
            "deferring": get("deferring"),
            "eventDriven": get("eventDriven"),
            "frequency": get("frequency"),
            "id": el.id,
            "implementationName": get("implementation"),
            "inheritBlackboard": get("inheritBlackboard"),
            "isInterface": get("isInterface"),
            "libraryName": el.library,
            "name": el.name,
            "postCondition": self.condition_object(spec.get("post"), el.where + ".post", q + "/post"),
            "preCondition": self.condition_object(spec.get("pre"), el.where + ".pre", q + "/pre"),
            "runtimeCondition": self.condition_object(spec.get("runtime"), el.where + ".runtime", q + "/runtime"),
            "variables": get("variables"),
        }

    def build_placeholder(self, el):
        if not self.check_keys(el.where, el.spec, {"id", "comment", "blackboard"}):
            return None
        return {
            "blackboard": self.blackboard(el.spec.get("blackboard"), el.where, "placeholder", f"plh:{el.name}"),
            "comment": el.spec.get("comment", ""),
            "id": el.id,
            "name": el.name,
        }

    def build_plantype(self, el):
        spec = el.spec
        if not self.check_keys(el.where, spec, {"id", "comment", "plans", "variables", "variableBindings"}):
            return None
        annotated = []
        plans = spec.get("plans") or []
        if not isinstance(plans, list):
            self.err(el.where, "plans must be a list")
            plans = []
        refs = []
        for item in plans:
            ref, opts = self.single(item, el.where + ".plans")
            if ref is None:
                continue
            refs.append((ref, opts))
        for (ref, opts), key in zip(refs, dup_keys([r for r, _ in refs], "#")):
            where = f"{el.where}.plans.{ref}"
            self.check_keys(where, opts, set(ANNOTATED_DEFAULTS) | {"id", "name"})
            refstr, target = self.ref_abstract(ref, where, ("plan",))
            default_name = annotated_default_name(target.name if target else None)
            annotated.append(
                {
                    "activated": opts.get("activated", ANNOTATED_DEFAULTS["activated"]),
                    "comment": opts.get("comment", ANNOTATED_DEFAULTS["comment"]),
                    "id": self.ident(opts, f"pty:{el.name}/plan:{key}"),
                    "name": opts.get("name", default_name),
                    "plan": refstr,
                }
            )
        return {
            "annotatedPlans": annotated,
            "comment": spec.get("comment", ""),
            "id": el.id,
            "name": el.name,
            "variableBindings": self.bindings(spec.get("variableBindings", []), el.where),
            "variables": spec.get("variables", []),
        }

    def build_taskrepository(self, el):
        if not self.check_keys(el.where, el.spec, {"id", "name", "comment", "tasks"}):
            return None
        return {
            "comment": el.spec.get("comment", ""),
            "id": el.id,
            "name": el.name,
            "tasks": [
                {"comment": n.spec.get("comment", ""), "id": n.id, "name": n.name, "taskRepository": el.id} for n in el.nested.values()
            ],
        }

    def build_conditionrepository(self, el):
        if not self.check_keys(el.where, el.spec, {"id", "name", "comment", "conditions"}):
            return None
        conditions = []
        for n in el.nested.values():
            library = n.spec.get("library", el.module.data.get("library", ""))
            conditions.append(
                {
                    "blackboard": self.blackboard(n.spec.get("blackboard"), f"{el.where}.conditions.{n.name}", "condition", f"cnd:{el.name}/condition:{n.name}"),
                    "comment": n.spec.get("comment", ""),
                    "conditionRepository": el.id,
                    "id": n.id,
                    "libraryName": library,
                    "name": n.name,
                }
            )
        return {"comment": el.spec.get("comment", ""), "conditions": conditions, "id": el.id, "name": el.name}

    def build_roleset(self, el):
        spec = el.spec
        if not self.check_keys(el.where, spec, set(ROLESET_DEFAULTS) | {"id", "name", "roles"}):
            return None
        roles = []
        role_specs = spec.get("roles") or {}
        if not isinstance(role_specs, dict):
            self.err(el.where, "roles must be a mapping of name to definition")
            role_specs = {}
        for name, rspec in role_specs.items():
            rspec = {} if rspec is None else rspec
            where = f"{el.where}.roles.{name}"
            if not self.check_keys(where, rspec, set(ROLE_DEFAULTS) | {"id"}):
                continue
            priorities = {}
            prio_spec = rspec.get("priorities") or {}
            if not isinstance(prio_spec, dict):
                self.err(where, "priorities must map task to number")
                prio_spec = {}
            for task, value in prio_spec.items():
                refstr, _ = self.ref_nested(task, where, self.tasks, "task")
                if refstr:
                    priorities[refstr] = value
            roles.append(
                {
                    "characteristics": rspec.get("characteristics", ROLE_DEFAULTS["characteristics"]),
                    "comment": rspec.get("comment", ROLE_DEFAULTS["comment"]),
                    "id": self.ident(rspec, f"rst:{el.name}/role:{name}"),
                    "name": name,
                    "roleSet": el.id,
                    "taskPriorities": priorities,
                }
            )
        return {
            "comment": spec.get("comment", ROLESET_DEFAULTS["comment"]),
            "defaultPriority": spec.get("defaultPriority", ROLESET_DEFAULTS["defaultPriority"]),
            "defaultRoleSet": spec.get("defaultRoleSet", ROLESET_DEFAULTS["defaultRoleSet"]),
            "id": el.id,
            "name": el.name,
            "roles": roles,
        }

    def build_plan(self, el):
        spec = el.spec
        allowed = set(PLAN_DEFAULTS) | {"id", "library", "blackboard", "entryPoints", "states", "transitions", "synchronisations"}
        if not self.check_keys(el.where, spec, allowed):
            return None
        q = f"pml:{el.name}"
        layout = el.module.layout.get(el.name) or {}
        if not isinstance(layout, dict):
            self.err(layout_path(el.module.relpath), f"layout of {el.name} must be a mapping")
            layout = {}
        get = lambda k: spec.get(k, PLAN_DEFAULTS[k])  # noqa: E731

        states_spec = spec.get("states") or {}
        if not isinstance(states_spec, dict):
            self.err(el.where, "states must be a mapping of state name to definition")
            states_spec = {}
        state_ids = {}
        state_bodies = {}
        for key, body in states_spec.items():
            if not isinstance(key, str) or not key or TRANSITION_SEP in key:
                self.err(el.where, f"invalid state name {key!r}")
                continue
            if body is None:
                body = {}
            elif isinstance(body, list):
                body = {"children": body}
            if not isinstance(body, dict):
                self.err(f"{el.where}.states.{key}", "state must be a list of children or a mapping")
                continue
            state_bodies[key] = body
            state_ids[key] = self.ident(body, f"{q}/state:{key}")

        eps_spec = spec.get("entryPoints") or []
        if not isinstance(eps_spec, list):
            self.err(el.where, "entryPoints must be a list")
            eps_spec = []
        entry_points = []
        ep_task_keys = [ep.get("task") if isinstance(ep, dict) else None for ep in eps_spec]
        ep_keys = dup_keys([str(k) for k in ep_task_keys], " #")
        ep_of_state = {}
        ep_layout = layout.get("entryPoints") or {}
        for index, (ep, ep_key) in enumerate(zip(eps_spec, ep_keys)):
            where = f"{el.where}.entryPoints[{index}]"
            if not self.check_keys(where, ep, set(EP_DEFAULTS) | {"id", "task", "state"}):
                continue
            task_ref, _ = self.ref_nested(ep.get("task"), where, self.tasks, "task")
            state_key = ep.get("state")
            if state_key not in state_ids:
                self.err(where, f"initial state {state_key!r} is not a state of this plan")
                continue
            ep_id = self.ident(ep, f"{q}/ep:{ep_key}")
            ep_of_state.setdefault(state_ids[state_key], ep_id)
            max_card = ep.get("max", EP_DEFAULTS["max"])
            if isinstance(max_card, str) and max_card.lower() in ("inf", "infinity", "unbounded"):
                max_card = INT_MAX
            elif isinstance(max_card, float) and math.isinf(max_card):
                max_card = INT_MAX
            entry_points.append(
                {
                    "comment": ep.get("comment", EP_DEFAULTS["comment"]),
                    "id": ep_id,
                    "isDynamic": ep.get("isDynamic", EP_DEFAULTS["isDynamic"]),
                    "maxCardinality": max_card,
                    "minCardinality": ep.get("min", EP_DEFAULTS["min"]),
                    "name": ep.get("name", EP_DEFAULTS["name"]),
                    "plan": el.id,
                    "positionWeb": self.position(ep_layout.get(ep_key), index, "ep"),
                    "state": state_ids[state_key],
                    "successRequired": ep.get("successRequired", EP_DEFAULTS["successRequired"]),
                    "task": task_ref,
                }
            )

        scopes = {"plan": el.id}
        scopes.update({f"state:{key}": sid for key, sid in state_ids.items()})
        for ep_key, ep in zip(ep_keys, eps_spec):
            if isinstance(ep, dict):
                scopes[f"ep:{ep_key}"] = self.ident(ep, f"{q}/ep:{ep_key}")

        trans_spec = spec.get("transitions") or []
        if not isinstance(trans_spec, list):
            self.err(el.where, "transitions must be a list")
            trans_spec = []
        parsed = []
        for index, item in enumerate(trans_spec):
            where = f"{el.where}.transitions[{index}]"
            label, body = self.single(item, where)
            if label is None:
                continue
            src, sep, dst = label.partition(TRANSITION_SEP)
            if not sep or src not in state_ids or dst not in state_ids:
                self.err(where, f"'{label}' must be 'From -> To' with states of this plan")
                continue
            if isinstance(body, str):
                body = {"condition": body}
            if not self.check_keys(where, body, set(TRANSITION_DEFAULTS) | {"id", "condition", "in", "out", "pre"}):
                continue
            parsed.append((label, src, dst, body, where))
        trans_keys = dup_keys([p[0] for p in parsed], " #")
        transitions = []
        trans_by_key = {}
        trans_layout = layout.get("transitions") or {}
        for (label, src, dst, body, where), tkey in zip(parsed, trans_keys):
            tq = f"{q}/transition:{tkey}"
            cond_ref = None
            cond = None
            if body.get("condition") is not None:
                cond_ref, cond = self.ref_nested(body["condition"], where, self.conditions, "condition")
            elif "condition" not in body:
                self.err(where, "transition needs a condition")
            if "pre" in body and body["pre"] is None:
                pre = None
            else:
                pre = self.condition_object(body.get("pre") or {}, where + ".pre", tq + "/pre", scopes)
            points = trans_layout.get(tkey) or []
            tr = {
                "comment": body.get("comment", TRANSITION_DEFAULTS["comment"]),
                "condition": cond_ref,
                "id": self.ident(body, tq),
                "inState": state_ids[src],
                "keyMapping": self.key_mapping(body.get("in"), body.get("out"), where),
                "name": body.get("name", TRANSITION_DEFAULTS["name"]),
                "outState": state_ids[dst],
                "pointsWeb": [self.point(p) for p in points],
                "preCondition": pre,
                "synchronisation": None,
            }
            transitions.append(tr)
            trans_by_key[tkey] = tr

        syncs = []
        sync_spec = spec.get("synchronisations") or {}
        if not isinstance(sync_spec, dict):
            self.err(el.where, "synchronisations must be a mapping of name to definition")
            sync_spec = {}
        sync_layout = layout.get("synchronisations") or {}
        for index, (key, body) in enumerate(sync_spec.items()):
            where = f"{el.where}.synchronisations.{key}"
            body = {} if body is None else body
            if not self.check_keys(where, body, set(SYNC_DEFAULTS) | {"id", "name", "transitions"}):
                continue
            sync_id = self.ident(body, f"{q}/sync:{key}")
            synced = []
            for tkey in body.get("transitions") or []:
                tr = trans_by_key.get(tkey)
                if tr is None:
                    self.err(where, f"no transition '{tkey}' in this plan")
                    continue
                if tr["synchronisation"] is not None:
                    self.err(where, f"transition '{tkey}' is already in another synchronisation")
                tr["synchronisation"] = sync_id
                synced.append(tr["id"])
            syncs.append(
                {
                    "comment": body.get("comment", SYNC_DEFAULTS["comment"]),
                    "failOnSyncTimeout": body.get("failOnSyncTimeout", SYNC_DEFAULTS["failOnSyncTimeout"]),
                    "id": sync_id,
                    "name": body.get("name", key),
                    "plan": el.id,
                    "positionWeb": self.position(sync_layout.get(key), index, "sync"),
                    "syncTimeout": body.get("syncTimeout", SYNC_DEFAULTS["syncTimeout"]),
                    "syncedTransitions": synced,
                    "talkTimeout": body.get("talkTimeout", SYNC_DEFAULTS["talkTimeout"]),
                }
            )

        states = []
        state_layout = layout.get("states") or {}
        for index, (key, body) in enumerate(state_bodies.items()):
            where = f"{el.where}.states.{key}"
            allowed = {"id", "name", "comment", "children", "terminal", "post", "variableBindings"}
            if not self.check_keys(where, body, allowed):
                continue
            sid = state_ids[key]
            terminal = body.get("terminal")
            keyword = TERMINAL_KEY.match(key)
            if terminal is None and keyword:
                terminal = keyword.group(1)
            if terminal not in (None, "success", "failure"):
                self.err(where, f"terminal must be success or failure, got {terminal!r}")
                terminal = None
            if keyword and terminal != keyword.group(1):
                self.err(where, f"state '{key}' is named like a {keyword.group(1)} state but is not one")
            default_name = terminal_default_name(key) if keyword else key
            children = body.get("children") or []
            if terminal and children:
                self.err(where, "terminal states cannot run plans or behaviours")
            if not isinstance(children, list):
                self.err(where, "children must be a list")
                children = []
            parsed_children = []
            for item in children:
                ref, opts = self.single(item, where)
                if ref is not None:
                    parsed_children.append((ref, opts))
            wrappers = []
            for (ref, opts), ckey in zip(parsed_children, dup_keys([r for r, _ in parsed_children], "#")):
                cwhere = f"{where}.{ref}"
                if not self.check_keys(cwhere, opts, set(WRAPPER_DEFAULTS) | {"id", "in", "out"}):
                    continue
                refstr, _ = self.ref_abstract(ref, cwhere)
                wrappers.append(
                    {
                        "abstractPlan": refstr,
                        "comment": opts.get("comment", WRAPPER_DEFAULTS["comment"]),
                        "configuration": opts.get("configuration", WRAPPER_DEFAULTS["configuration"]),
                        "id": self.ident(opts, f"{q}/state:{key}/child:{ckey}"),
                        "keyMapping": self.key_mapping(opts.get("in"), opts.get("out"), cwhere),
                        "name": opts.get("name", WRAPPER_DEFAULTS["name"]),
                    }
                )
            if not terminal and "post" in body:
                self.err(where, "only terminal states have a post condition")
            state = {
                "comment": body.get("comment", ""),
                "confAbstractPlanWrappers": wrappers,
                "entryPoint": ep_of_state.get(sid),
                "id": sid,
                "inTransitions": [t["id"] for t in transitions if t["outState"] == sid],
                "name": body.get("name", default_name),
                "outTransitions": [t["id"] for t in transitions if t["inState"] == sid],
                "parentPlan": el.id,
                "positionWeb": self.position(state_layout.get(key), index, "state"),
                "type": "TerminalState" if terminal else "State",
                "variableBindings": self.bindings(body.get("variableBindings", []), where),
            }
            if terminal:
                state["postCondition"] = self.condition_object(body.get("post"), where + ".post", f"{q}/state:{key}/post", scopes)
                state["success"] = terminal == "success"
            states.append(state)

        return {
            "blackboard": self.blackboard(spec.get("blackboard"), el.where, "plan", q),
            "comment": get("comment"),
            "entryPoints": entry_points,
            "frequency": get("frequency"),
            "id": el.id,
            "implementationName": get("implementation"),
            "inheritBlackboard": get("inheritBlackboard"),
            "isInterface": get("isInterface"),
            "libraryName": el.library,
            "masterPlan": get("master"),
            "name": el.name,
            "preCondition": self.condition_object(spec.get("pre"), el.where + ".pre", q + "/pre", scopes),
            "runtimeCondition": self.condition_object(spec.get("runtime"), el.where + ".runtime", q + "/runtime", scopes),
            "states": states,
            "synchronisations": syncs,
            "transitions": transitions,
            "utilityThreshold": get("utilityThreshold"),
            "variables": get("variables"),
        }

    def position(self, value, index, kind):
        if value is not None:
            if isinstance(value, list) and len(value) == 2:
                return {"x": value[0], "y": value[1]}
            self.err("layout", f"position must be [x, y], got {value!r}")
        if kind == "ep":
            return {"x": 100, "y": 100 + 150 * index}
        if kind == "sync":
            return {"x": 300 + 200 * index, "y": 20}
        return {"x": 300 + 220 * (index % 5), "y": 100 + 150 * (index // 5)}

    def point(self, value):
        if isinstance(value, list) and len(value) == 2:
            return {"x": value[0], "y": value[1]}
        self.err("layout", f"transition point must be [x, y], got {value!r}")
        return {"x": 0, "y": 0}


def load_modules(src_root):
    modules = []
    errors = []
    for rel in source_files(src_root):
        path = os.path.join(src_root, rel)
        try:
            data = load_yaml(path)
        except Exception as e:
            errors.append(f"{rel}: {e}")
            continue
        layout = {}
        lpath = layout_path(path)
        if os.path.exists(lpath):
            try:
                layout = load_yaml(lpath) or {}
            except Exception as e:
                errors.append(f"{os.path.relpath(lpath, src_root)}: {e}")
        modules.append(Module(rel.replace(os.sep, "/"), data, layout if isinstance(layout, dict) else {}))
    if errors:
        raise CompileError(errors)
    return modules


def compile_dir(src_root):
    return Compiler(load_modules(src_root)).compile()
