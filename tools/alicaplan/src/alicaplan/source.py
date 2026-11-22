import hashlib
import os
import re

import yaml

from .jsonio import INT_MAX

KIND_EXT = {
    "plan": ".pml",
    "behaviour": ".beh",
    "plantype": ".pty",
    "placeholder": ".plh",
    "taskrepository": ".tsk",
    "conditionrepository": ".cnd",
    "roleset": ".rst",
}
EXT_KIND = {v: k for k, v in KIND_EXT.items()}

SECTION_KIND = {
    "plans": "plan",
    "behaviours": "behaviour",
    "plantypes": "plantype",
    "placeholders": "placeholder",
}
REPO_SECTION_KIND = {
    "task_repository": "taskrepository",
    "condition_repository": "conditionrepository",
    "roleset": "roleset",
}
REPO_DEFAULT_NAME = {
    "taskrepository": "TaskRepository",
    "conditionrepository": "ConditionRepository",
    "roleset": "RoleSet",
}
MODULE_KEYS = {"library"} | set(SECTION_KIND) | set(REPO_SECTION_KIND)

TERMINAL_KEY = re.compile(r"^(success|failure)(?:\.([2-9]|[1-9][0-9]+))?$")
TRANSITION_SEP = " -> "

DEFAULT_ACCESS = {"plan": "protected", "behaviour": "input", "condition": "input", "placeholder": "input"}

PLAN_DEFAULTS = {
    "comment": "",
    "frequency": 0,
    "implementation": "",
    "inheritBlackboard": False,
    "isInterface": False,
    "master": False,
    "utilityThreshold": 0.1,
    "variables": [],
    "pre": None,
    "runtime": None,
}
BEHAVIOUR_DEFAULTS = {
    "comment": "",
    "deferring": 0,
    "eventDriven": False,
    "frequency": 30,
    "implementation": "",
    "inheritBlackboard": False,
    "isInterface": False,
    "variables": [],
    "pre": None,
    "runtime": None,
    "post": None,
}
EP_DEFAULTS = {"min": 0, "max": INT_MAX, "successRequired": False, "isDynamic": False, "name": "", "comment": ""}
WRAPPER_DEFAULTS = {"comment": "", "name": "", "configuration": None}
TRANSITION_DEFAULTS = {"comment": "", "name": ""}
SYNC_DEFAULTS = {"failOnSyncTimeout": False, "syncTimeout": 10000, "talkTimeout": 30, "comment": ""}
COND_DEFAULTS = {"name": "", "comment": "", "conditionString": "", "enabled": True, "quantifiers": [], "variables": []}
BB_DEFAULTS = {"default": None, "comment": ""}
ANNOTATED_DEFAULTS = {"activated": True, "comment": ""}
ROLESET_DEFAULTS = {"comment": "", "defaultPriority": 0.5, "defaultRoleSet": True}
ROLE_DEFAULTS = {"comment": "", "characteristics": [], "priorities": {}}


def hash_id(qualified):
    digest = hashlib.sha256(qualified.encode("utf-8")).digest()
    return int.from_bytes(digest[:8], "big") >> 1


def terminal_default_name(key):
    match = TERMINAL_KEY.match(key)
    if not match:
        return None
    return match.group(1).capitalize() + (match.group(2) or "")


def annotated_default_name(plan_name):
    return f"Annotated{plan_name}" if plan_name else ""


def same(a, b):
    if type(a) is not type(b):
        return False
    return a == b


def dup_keys(keys, sep):
    counts = {}
    result = []
    for key in keys:
        counts[key] = counts.get(key, 0) + 1
        result.append(key if counts[key] == 1 else f"{key}{sep}{counts[key]}")
    return result


class _Loader(yaml.SafeLoader):
    pass


def _no_duplicates(loader, node, deep=False):
    keys = set()
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            hashable = key in keys
        except TypeError:
            continue
        if hashable:
            raise yaml.constructor.ConstructorError(None, None, f"duplicate key {key!r}", key_node.start_mark)
        keys.add(key)
    return loader.construct_mapping(node, deep)


_Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _no_duplicates)


def load_yaml(path):
    with open(path) as f:
        return yaml.load(f, Loader=_Loader)


class _Dumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def _repr_list(dumper, data):
    flow = all(not isinstance(x, (dict, list)) for x in data) or all(
        isinstance(x, list) and all(not isinstance(y, (dict, list)) for y in x) for x in data
    )
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=flow and len(data) > 0)


def _repr_str(dumper, data):
    if "\n" in data:
        return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")
    return dumper.represent_scalar("tag:yaml.org,2002:str", data)


_Dumper.add_representer(list, _repr_list)
_Dumper.add_representer(str, _repr_str)


def dump_yaml(data):
    return yaml.dump(data, Dumper=_Dumper, sort_keys=False, default_flow_style=False, allow_unicode=True, width=120)


MODULE_SUFFIX = ".alica.yaml"
LAYOUT_SUFFIX = ".layout.yaml"


def source_files(root):
    found = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames.sort()
        for name in sorted(filenames):
            if name.endswith(MODULE_SUFFIX):
                found.append(os.path.relpath(os.path.join(dirpath, name), root))
    return found


def layout_path(module_path):
    return module_path[: -len(MODULE_SUFFIX)] + LAYOUT_SUFFIX
