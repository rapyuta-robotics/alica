import os
from collections import Counter, defaultdict, deque
from dataclasses import dataclass

from .jsonio import EXTENSIONS

BLACKBOARD_TYPES = {"bool", "int64", "uint64", "double", "std::string", "std::any"}
ACCESS_TYPES = {"input", "output", "protected"}
ABSTRACT_PLAN_KINDS = {"plan", "behaviour", "plantype", "placeholder"}


@dataclass(frozen=True)
class Finding:
    severity: str
    code: str
    file: str
    where: str
    message: str

    def __str__(self):
        where = f" {self.where}:" if self.where else ""
        return f"{self.severity}: {self.file}:{where} {self.message} [{self.code}]"


@dataclass
class Entry:
    id: int
    kind: str
    doc: object
    obj: dict
    owner: object


def describe(kind, obj):
    name = obj.get("name") if isinstance(obj, dict) else None
    label = f"'{name}'" if name else f"#{obj.get('id')}" if isinstance(obj, dict) else "?"
    return f"{kind} {label}"


class Checker:
    def __init__(self, docs):
        self.docs = docs
        self.findings = []
        self.elements = {}
        self._seen_ids = defaultdict(list)

    def add(self, severity, code, doc, where, message):
        self.findings.append(Finding(severity, code, doc.relpath, where, message))

    def error(self, code, doc, where, message):
        self.add("error", code, doc, where, message)

    def warning(self, code, doc, where, message):
        self.add("warning", code, doc, where, message)

    def run(self):
        for doc in self.docs:
            for message in doc.errors:
                self.error("parse", doc, "", message)
        docs = [d for d in self.docs if not d.errors]
        self.check_basenames(docs)
        for doc in docs:
            self.register_doc(doc)
        for elem_id, entries in self._seen_ids.items():
            if len(entries) > 1:
                places = ", ".join(f"{e.doc.relpath} ({describe(e.kind, e.obj)})" for e in entries)
                self.error("duplicate-id", entries[0].doc, "", f"id {elem_id} is used more than once: {places}")
        for doc in docs:
            getattr(self, "check_" + doc.kind)(doc)
        order = {"error": 0, "warning": 1}
        self.findings.sort(key=lambda f: (f.file, order[f.severity], f.code, f.where, f.message))
        return self.findings

    def check_basenames(self, docs):
        seen = defaultdict(list)
        for doc in docs:
            seen[doc.basename].append(doc)
        for name, same in seen.items():
            if len(same) > 1:
                others = ", ".join(os.path.join(d.root, d.relpath) for d in same)
                self.error("duplicate-file", same[0], "", f"file name {name} exists more than once ({others}); the engine loads whichever it finds first")

    def register(self, doc, kind, obj, owner, where):
        if not isinstance(obj, dict):
            self.error("schema", doc, where, f"{kind} must be an object")
            return None
        elem_id = obj.get("id")
        if not isinstance(elem_id, int) or isinstance(elem_id, bool):
            self.error("bad-id", doc, where, f"{describe(kind, obj)} has no integer id")
            return None
        entry = Entry(elem_id, kind, doc, obj, owner)
        self._seen_ids[elem_id].append(entry)
        self.elements.setdefault(elem_id, entry)
        return entry

    def items(self, doc, obj, key, where):
        value = obj.get(key)
        if value is None:
            return []
        if not isinstance(value, list):
            self.error("schema", doc, where, f"'{key}' must be a list")
            return []
        return value

    def register_condition_parts(self, doc, cond, owner, where):
        for quant in self.items(doc, cond, "quantifiers", where):
            self.register(doc, "quantifier", quant, owner, where)

    def register_doc(self, doc):
        data = doc.data
        root = self.register(doc, doc.kind, data, None, "")
        if root is None:
            return
        if data.get("name") != doc.stem:
            self.warning("name-mismatch", doc, "", f"element name '{data.get('name')}' differs from file name {doc.basename}")
        for var in self.items(doc, data, "variables", ""):
            self.register(doc, "variable", var, root, "")
        for key in ("preCondition", "runtimeCondition", "postCondition"):
            cond = data.get(key)
            if cond is not None:
                entry = self.register(doc, key.lower(), cond, root, key)
                if entry:
                    self.register_condition_parts(doc, cond, entry, key)
        if doc.kind == "plan":
            for ep in self.items(doc, data, "entryPoints", ""):
                self.register(doc, "entrypoint", ep, root, describe("entry point", ep))
            for state in self.items(doc, data, "states", ""):
                where = describe("state", state)
                kind = "terminalstate" if isinstance(state, dict) and state.get("type") == "TerminalState" else "state"
                sentry = self.register(doc, kind, state, root, where)
                if sentry is None:
                    continue
                for wrapper in self.items(doc, state, "confAbstractPlanWrappers", where):
                    self.register(doc, "wrapper", wrapper, sentry, where)
                for binding in self.items(doc, state, "variableBindings", where):
                    self.register(doc, "variablebinding", binding, sentry, where)
                post = state.get("postCondition")
                if post is not None:
                    pentry = self.register(doc, "postcondition", post, sentry, where)
                    if pentry:
                        self.register_condition_parts(doc, post, pentry, where)
            for tr in self.items(doc, data, "transitions", ""):
                where = describe("transition", tr)
                tentry = self.register(doc, "transition", tr, root, where)
                if tentry and tr.get("preCondition") is not None:
                    pentry = self.register(doc, "precondition", tr["preCondition"], tentry, where)
                    if pentry:
                        self.register_condition_parts(doc, tr["preCondition"], pentry, where)
            for sync in self.items(doc, data, "synchronisations", ""):
                self.register(doc, "synchronisation", sync, root, describe("synchronisation", sync))
        elif doc.kind == "plantype":
            for ap in self.items(doc, data, "annotatedPlans", ""):
                self.register(doc, "annotatedplan", ap, root, "")
            for binding in self.items(doc, data, "variableBindings", ""):
                self.register(doc, "variablebinding", binding, root, "")
        elif doc.kind == "taskrepository":
            for task in self.items(doc, data, "tasks", ""):
                self.register(doc, "task", task, root, describe("task", task))
        elif doc.kind == "conditionrepository":
            for cond in self.items(doc, data, "conditions", ""):
                self.register(doc, "condition", cond, root, describe("condition", cond))
        elif doc.kind == "roleset":
            for role in self.items(doc, data, "roles", ""):
                self.register(doc, "role", role, root, describe("role", role))

    def resolve(self, doc, where, ref, kinds, what):
        if ref is None:
            self.error("ref-missing", doc, where, f"{what} is not set")
            return None
        if isinstance(ref, bool):
            self.error("ref-malformed", doc, where, f"{what} has malformed reference {ref!r}")
            return None
        if isinstance(ref, int):
            file_part, id_part = "", str(ref)
        elif isinstance(ref, str):
            file_part, _, id_part = ref.rpartition("#")
        else:
            self.error("ref-malformed", doc, where, f"{what} has malformed reference {ref!r}")
            return None
        try:
            elem_id = int(id_part)
        except ValueError:
            self.error("ref-malformed", doc, where, f"{what} has malformed reference {ref!r}")
            return None
        entry = self.elements.get(elem_id)
        if entry is None:
            self.error("ref-unresolved", doc, where, f"{what} {ref!r} does not match any element")
            return None
        if file_part:
            base = file_part.replace("\\", "/").rsplit("/", 1)[-1]
            if os.path.splitext(base)[1] not in EXTENSIONS:
                self.error("ref-wrong-file", doc, where, f"{what} {ref!r} names a file with an unknown extension")
            elif base != entry.doc.basename:
                self.error("ref-wrong-file", doc, where, f"{what} {ref!r} names file {base}, but id {elem_id} is defined in {entry.doc.basename}")
        elif entry.doc is not doc:
            self.error("ref-wrong-file", doc, where, f"{what} {ref!r} has no file name, but id {elem_id} is defined in {entry.doc.basename}")
        if entry.kind not in kinds:
            self.error("ref-wrong-kind", doc, where, f"{what} {ref!r} points to {describe(entry.kind, entry.obj)}, expected {' or '.join(sorted(kinds))}")
            return None
        return entry

    def blackboard(self, doc, owner, where):
        entries = {}
        for item in self.items(doc, owner, "blackboard", where):
            if not isinstance(item, dict):
                self.error("schema", doc, where, "blackboard entries must be objects")
                continue
            key = item.get("key")
            if not isinstance(key, str) or not key:
                self.error("blackboard", doc, where, "blackboard entry without a key")
                continue
            if key in entries:
                self.error("blackboard", doc, where, f"blackboard key '{key}' is defined twice")
            if item.get("type") not in BLACKBOARD_TYPES:
                self.error("blackboard", doc, where, f"blackboard key '{key}' has unsupported type {item.get('type')!r}")
            if item.get("access") not in ACCESS_TYPES:
                self.error("blackboard", doc, where, f"blackboard key '{key}' has unknown access {item.get('access')!r}")
            entries[key] = item
        return entries

    def check_conditions_of(self, doc, data):
        for key in ("preCondition", "runtimeCondition", "postCondition"):
            cond = data.get(key)
            if cond is not None:
                self.check_condition_object(doc, cond, key)

    def check_condition_object(self, doc, cond, where):
        for var in self.items(doc, cond, "variables", where):
            self.resolve(doc, where, var, {"variable"}, "condition variable")
        for quant in self.items(doc, cond, "quantifiers", where):
            if isinstance(quant, dict):
                scope = quant.get("scope")
                self.resolve(doc, where, scope, {"plan", "entrypoint", "state", "terminalstate"}, "quantifier scope")

    def check_bindings(self, doc, holder, where):
        for binding in self.items(doc, holder, "variableBindings", where):
            if not isinstance(binding, dict):
                continue
            self.resolve(doc, where, binding.get("subPlan"), ABSTRACT_PLAN_KINDS, "variable binding subPlan")
            self.resolve(doc, where, binding.get("subVariable"), {"variable"}, "variable binding subVariable")
            self.resolve(doc, where, binding.get("variable"), {"variable"}, "variable binding variable")

    def check_behaviour(self, doc):
        self.blackboard(doc, doc.data, "")
        self.check_conditions_of(doc, doc.data)

    def check_placeholder(self, doc):
        self.blackboard(doc, doc.data, "")

    def check_plantype(self, doc):
        plans = set()
        for ap in self.items(doc, doc.data, "annotatedPlans", ""):
            if not isinstance(ap, dict):
                continue
            entry = self.resolve(doc, describe("annotated plan", ap), ap.get("plan"), {"plan"}, "annotated plan")
            if entry:
                if entry.id in plans:
                    self.error("plantype", doc, "", f"plan {entry.obj.get('name')} is annotated twice")
                plans.add(entry.id)
        if not plans:
            self.warning("plantype", doc, "", "plan type has no plans")
        self.check_bindings(doc, doc.data, "")

    def check_taskrepository(self, doc):
        names = Counter()
        for task in self.items(doc, doc.data, "tasks", ""):
            if not isinstance(task, dict):
                continue
            names[task.get("name")] += 1
            if task.get("taskRepository") != doc.data.get("id"):
                self.error("back-ref", doc, describe("task", task), f"taskRepository {task.get('taskRepository')} should be {doc.data.get('id')}")
        for name, count in names.items():
            if count > 1:
                self.error("duplicate-name", doc, "", f"task name '{name}' is used {count} times")

    def check_conditionrepository(self, doc):
        names = Counter()
        for cond in self.items(doc, doc.data, "conditions", ""):
            if not isinstance(cond, dict):
                continue
            where = describe("condition", cond)
            names[cond.get("name")] += 1
            if cond.get("conditionRepository") != doc.data.get("id"):
                self.error("back-ref", doc, where, f"conditionRepository {cond.get('conditionRepository')} should be {doc.data.get('id')}")
            self.blackboard(doc, cond, where)
        for name, count in names.items():
            if count > 1:
                self.error("duplicate-name", doc, "", f"condition name '{name}' is used {count} times")

    def check_roleset(self, doc):
        names = Counter()
        for role in self.items(doc, doc.data, "roles", ""):
            if not isinstance(role, dict):
                continue
            where = describe("role", role)
            names[role.get("name")] += 1
            if role.get("roleSet") != doc.data.get("id"):
                self.error("back-ref", doc, where, f"roleSet {role.get('roleSet')} should be {doc.data.get('id')}")
            priorities = role.get("taskPriorities") or {}
            if not isinstance(priorities, dict):
                self.error("schema", doc, where, "taskPriorities must be an object")
                continue
            for ref, value in priorities.items():
                self.resolve(doc, where, ref, {"task"}, "task priority")
                if not isinstance(value, (int, float)) or isinstance(value, bool):
                    self.error("roleset", doc, where, f"priority for {ref} is not a number")
        for name, count in names.items():
            if count > 1:
                self.error("duplicate-name", doc, "", f"role name '{name}' is used {count} times")

    def child_blackboards(self, entry):
        if entry.kind == "plantype":
            boards = []
            for ap in entry.obj.get("annotatedPlans") or []:
                plan = self.elements.get(self.ref_id(ap.get("plan")))
                if plan is None or plan.kind != "plan":
                    continue
                boards.append((plan.obj.get("name"), plan.doc, plan.obj))
            return boards
        if entry.kind == "condition":
            return [(entry.obj.get("name"), entry.doc, entry.obj)]
        return [(entry.obj.get("name"), entry.doc, entry.obj)]

    @staticmethod
    def ref_id(ref):
        try:
            if isinstance(ref, int) and not isinstance(ref, bool):
                return ref
            return int(str(ref).rpartition("#")[2])
        except ValueError:
            return None

    def check_key_mapping(self, doc, where, mapping, parent, child_entry):
        if mapping is None:
            return
        if not isinstance(mapping, dict):
            self.error("schema", doc, where, "keyMapping must be an object")
            return
        parent_bb = None if parent.get("inheritBlackboard") else {e.get("key"): e for e in parent.get("blackboard") or [] if isinstance(e, dict)}
        children = []
        for name, cdoc, cobj in self.child_blackboards(child_entry):
            bb = None if cobj.get("inheritBlackboard") else {e.get("key"): e for e in cobj.get("blackboard") or [] if isinstance(e, dict)}
            children.append((name, bb))
        child_label = child_entry.obj.get("name")

        seen_inputs = set()
        for item in self.items(doc, mapping, "input", where):
            if not isinstance(item, dict):
                continue
            child_key, parent_key, value = item.get("childKey"), item.get("parentKey"), item.get("value")
            if not isinstance(child_key, str):
                self.error("key-mapping", doc, where, "input mapping without childKey")
                continue
            if child_key in seen_inputs:
                self.error("key-mapping", doc, where, f"child key '{child_key}' of {child_label} gets more than one input")
            seen_inputs.add(child_key)
            if (parent_key is None) == (value is None):
                self.error("key-mapping", doc, where, f"input to '{child_key}' must set exactly one of parentKey and value")
                continue
            if parent_key is not None:
                ptype = self.key_type(doc, where, parent_bb, parent_key, f"parent key '{parent_key}'", parent.get("name"))
            for name, bb in children:
                ctype = self.key_type(doc, where, bb, child_key, f"child key '{child_key}'", name)
                if ctype is None:
                    continue
                if parent_key is not None:
                    if ptype is not None and ctype != "std::any" and ptype != ctype:
                        self.error("key-type", doc, where, f"'{parent_key}' ({ptype}) is mapped into {name}.{child_key} ({ctype})")
                else:
                    self.check_constant(doc, where, name, child_key, ctype, value)

        seen_outputs = set()
        for item in self.items(doc, mapping, "output", where):
            if not isinstance(item, dict):
                continue
            child_key, parent_key = item.get("childKey"), item.get("parentKey")
            if not isinstance(child_key, str) or not isinstance(parent_key, str):
                self.error("key-mapping", doc, where, "output mapping needs childKey and parentKey")
                continue
            if parent_key in seen_outputs:
                self.error("key-mapping", doc, where, f"parent key '{parent_key}' gets more than one output from {child_label}")
            seen_outputs.add(parent_key)
            ptype = self.key_type(doc, where, parent_bb, parent_key, f"parent key '{parent_key}'", parent.get("name"))
            for name, bb in children:
                ctype = self.key_type(doc, where, bb, child_key, f"child key '{child_key}'", name)
                if ctype is not None and ptype is not None and ptype != "std::any" and ptype != ctype:
                    self.error("key-type", doc, where, f"{name}.{child_key} ({ctype}) is mapped into '{parent_key}' ({ptype})")

    def key_type(self, doc, where, bb, key, label, owner_name):
        if bb is None:
            return None
        entry = bb.get(key)
        if entry is None:
            self.error("key-missing", doc, where, f"{label} is not in the blackboard of {owner_name}")
            return None
        return entry.get("type")

    def check_constant(self, doc, where, name, key, ktype, value):
        if ktype == "std::any":
            self.error("key-type", doc, where, f"constant cannot be mapped into {name}.{key} because its type is std::any")
            return
        text = str(value).strip()
        ok = True
        if ktype == "bool":
            ok = isinstance(value, bool) or text.lower() in ("true", "false", "0", "1")
        elif ktype in ("int64", "uint64"):
            ok = not isinstance(value, float) and _is_int(text) and not (ktype == "uint64" and text.startswith("-"))
        elif ktype == "double":
            ok = _is_float(text) and not isinstance(value, bool)
        if not ok:
            self.error("key-type", doc, where, f"constant {value!r} is not a valid {ktype} for {name}.{key}")

    def check_plan(self, doc):
        plan = doc.data
        plan_id = plan.get("id")
        is_master = bool(plan.get("masterPlan"))
        self.blackboard(doc, plan, "")
        self.check_conditions_of(doc, plan)

        local = {}
        for kind in ("entryPoints", "states", "transitions", "synchronisations"):
            for obj in self.items(doc, plan, kind, ""):
                if isinstance(obj, dict) and isinstance(obj.get("id"), int):
                    local[obj["id"]] = (kind, obj)

        def local_ref(where, ref, kinds, what, allow_none=False):
            if ref is None:
                if not allow_none:
                    self.error("ref-missing", doc, where, f"{what} is not set")
                return None
            found = local.get(ref) if isinstance(ref, int) and not isinstance(ref, bool) else None
            if found is None or found[0] not in kinds:
                self.error("ref-unresolved", doc, where, f"{what} {ref!r} is not a {' or '.join(k[:-1] for k in kinds)} of this plan")
                return None
            return found[1]

        eps = [e for e in self.items(doc, plan, "entryPoints", "") if isinstance(e, dict)]
        states = [s for s in self.items(doc, plan, "states", "") if isinstance(s, dict)]
        transitions = [t for t in self.items(doc, plan, "transitions", "") if isinstance(t, dict)]
        syncs = [s for s in self.items(doc, plan, "synchronisations", "") if isinstance(s, dict)]

        if not eps:
            self.error("entry-point", doc, "", "plan has no entry points")
        ep_of_state = {}
        for ep in eps:
            where = describe("entry point", ep)
            self.resolve(doc, where, ep.get("task"), {"task"}, "task")
            if ep.get("plan") != plan_id:
                self.error("back-ref", doc, where, f"plan {ep.get('plan')} should be {plan_id}")
            state = local_ref(where, ep.get("state"), {"states"}, "initial state")
            if state is not None:
                if state.get("id") in ep_of_state:
                    self.error("entry-point", doc, where, f"state '{state.get('name')}' is already the initial state of another entry point")
                else:
                    ep_of_state[state.get("id")] = ep.get("id")
                if state.get("type") == "TerminalState":
                    self.error("entry-point", doc, where, "initial state is a terminal state")
            lo, hi = ep.get("minCardinality"), ep.get("maxCardinality")
            if not isinstance(lo, int) or not isinstance(hi, int) or lo < 0 or lo > hi:
                self.error("entry-point", doc, where, f"cardinality [{lo}..{hi}] is invalid")

        out_of = defaultdict(list)
        in_of = defaultdict(list)
        for tr in transitions:
            where = describe("transition", tr)
            src = local_ref(where, tr.get("inState"), {"states"}, "inState")
            dst = local_ref(where, tr.get("outState"), {"states"}, "outState")
            if src is not None:
                out_of[src["id"]].append(tr.get("id"))
            if dst is not None:
                in_of[dst["id"]].append(tr.get("id"))
            cond = self.resolve(doc, where, tr.get("condition"), {"condition"}, "condition")
            if cond is not None:
                self.check_key_mapping(doc, where, tr.get("keyMapping"), plan, cond)
            if tr.get("preCondition") is not None:
                self.check_condition_object(doc, tr["preCondition"], where)
            sync_id = tr.get("synchronisation")
            if sync_id is not None:
                sync = local_ref(where, sync_id, {"synchronisations"}, "synchronisation")
                if sync is not None and tr.get("id") not in (sync.get("syncedTransitions") or []):
                    self.error("back-ref", doc, where, f"synchronisation {sync_id} does not list this transition")

        for sync in syncs:
            where = describe("synchronisation", sync)
            if sync.get("plan") != plan_id:
                self.error("back-ref", doc, where, f"plan {sync.get('plan')} should be {plan_id}")
            synced = self.items(doc, sync, "syncedTransitions", where)
            for tid in synced:
                tr = local_ref(where, tid, {"transitions"}, "synced transition")
                if tr is not None and tr.get("synchronisation") != sync.get("id"):
                    self.error("back-ref", doc, where, f"transition {tid} does not point back to this synchronisation")
            if len(synced) < 2:
                self.warning("synchronisation", doc, where, "synchronisation has fewer than two transitions")

        terminal_ids = set()
        for state in states:
            where = describe("state", state)
            sid = state.get("id")
            terminal = state.get("type") == "TerminalState"
            if state.get("type") not in ("State", "TerminalState"):
                self.error("schema", doc, where, f"unknown state type {state.get('type')!r}")
            if state.get("parentPlan") != plan_id:
                self.error("back-ref", doc, where, f"parentPlan {state.get('parentPlan')} should be {plan_id}")
            if state.get("entryPoint") != ep_of_state.get(sid):
                self.error("back-ref", doc, where, f"entryPoint {state.get('entryPoint')} should be {ep_of_state.get(sid)}")
            if list(state.get("outTransitions") or []) != out_of.get(sid, []):
                self.error("back-ref", doc, where, f"outTransitions {state.get('outTransitions')} should be {out_of.get(sid, [])}")
            if list(state.get("inTransitions") or []) != in_of.get(sid, []):
                self.error("back-ref", doc, where, f"inTransitions {state.get('inTransitions')} should be {in_of.get(sid, [])}")
            wrappers = self.items(doc, state, "confAbstractPlanWrappers", where)
            if terminal:
                terminal_ids.add(sid)
                if out_of.get(sid):
                    self.error("terminal-state", doc, where, "terminal state has outgoing transitions")
                if wrappers:
                    self.error("terminal-state", doc, where, "terminal state runs plans or behaviours")
                if is_master:
                    kind = "success" if state.get("success") else "failure"
                    self.error("master-terminal", doc, where, f"master plan has a {kind} state; reaching it would end the agent's plan execution")
                if state.get("postCondition") is not None:
                    self.check_condition_object(doc, state["postCondition"], where)
            for wrapper in wrappers:
                if not isinstance(wrapper, dict):
                    continue
                child = self.resolve(doc, where, wrapper.get("abstractPlan"), ABSTRACT_PLAN_KINDS, "child")
                if child is not None:
                    self.check_key_mapping(doc, f"{where} -> {child.obj.get('name')}", wrapper.get("keyMapping"), plan, child)
            self.check_bindings(doc, state, where)

        names = Counter(s.get("name") for s in states if s.get("name"))
        for name, count in names.items():
            if count > 1:
                self.warning("duplicate-name", doc, "", f"state name '{name}' is used {count} times")

        succ = defaultdict(list)
        for tr in transitions:
            if tr.get("inState") in local and tr.get("outState") in local:
                succ[tr["inState"]].append(tr["outState"])
        start = [ep.get("state") for ep in eps if ep.get("state") in local]
        reached = set(start)
        queue = deque(start)
        while queue:
            for nxt in succ[queue.popleft()]:
                if nxt not in reached:
                    reached.add(nxt)
                    queue.append(nxt)
        for state in states:
            if state.get("id") not in reached:
                self.warning("unreachable", doc, describe("state", state), "state cannot be reached from any entry point")
        if not is_master and eps and not (reached & terminal_ids):
            self.warning("no-terminal", doc, "", "no success or failure state can be reached from an entry point")


def _is_int(text):
    try:
        int(text)
        return True
    except ValueError:
        return False


def _is_float(text):
    try:
        float(text)
        return True
    except ValueError:
        return False


def check_docs(docs):
    return Checker(docs).run()
