import copy

import pytest

from alicaplan.check import check_docs
from alicaplan.compiler import CompileError
from alicaplan.jsonio import load_dirs

from conftest import FIXTURES, codes, compile_modules, findings_for

BASE = {
    "library": "lib",
    "task_repository": {"tasks": ["T", "U"]},
    "condition_repository": {
        "conditions": {
            "C": {},
            "Keyed": {"blackboard": {"flag": "bool", "count": "int64"}},
        }
    },
    "behaviours": {
        "B": {"blackboard": {"speed": "double", "done": {"type": "bool", "access": "output"}, "anything": "std::any"}},
    },
    "plans": {
        "P": {
            "blackboard": {"v": "double", "ok": "bool", "n": "int64"},
            "entryPoints": [{"task": "T", "state": "A"}],
            "states": {"A": ["B"], "success": None},
            "transitions": [{"A -> success": "C"}],
        },
    },
}


def variant(edit):
    data = copy.deepcopy(BASE)
    edit(data)
    return findings_for((data, None))


def plan(data):
    return data["plans"]["P"]


def test_base_is_clean():
    assert findings_for((BASE, None)) == []


def test_fixture_errors_are_the_known_ones():
    errors = [(f.file, f.code) for f in check_docs(load_dirs([FIXTURES["alica_tests"]])) if f.severity == "error"]
    assert errors == [
        ("plans/FailureHandlingMaster.pml", "master-terminal"),
        ("plans/MasterPlan.pml", "master-terminal"),
    ] + [("plans/OrderedSchedulingTestPlan.pml", "ref-missing")] * 5 + [
        ("plans/StandardLibraryCompareConditionsPlan.pml", "master-terminal"),
    ]
    assert [f for f in check_docs(load_dirs([FIXTURES["alica_turtlesim"]])) if f.severity == "error"] == []


def test_master_plan_cannot_have_terminal_states():
    findings = variant(lambda d: plan(d).update(master=True))
    assert codes(findings, "error") == ["master-terminal"]


def test_master_plan_without_terminal_is_fine():
    def edit(d):
        plan(d).update(master=True, transitions=[])
        del plan(d)["states"]["success"]

    assert codes(variant(edit), "error") == []


def test_terminal_state_with_outgoing_transition():
    def edit(d):
        plan(d)["transitions"].append({"success -> A": "C"})

    assert "terminal-state" in codes(variant(edit), "error")


def test_plan_without_reachable_terminal_warns():
    def edit(d):
        plan(d)["states"]["Z"] = ["B"]
        plan(d)["transitions"] = [{"A -> Z": "C"}]
        del plan(d)["states"]["success"]

    assert codes(variant(edit)) == ["no-terminal"]


def test_unreachable_state_warns():
    def edit(d):
        plan(d)["states"]["Orphan"] = ["B"]

    findings = variant(edit)
    assert codes(findings) == ["unreachable"]
    assert findings[0].where == "state 'Orphan'"


def test_bad_cardinality():
    def edit(d):
        plan(d)["entryPoints"][0].update(min=3, max=1)

    assert codes(variant(edit), "error") == ["entry-point"]


def test_two_entry_points_on_one_state():
    def edit(d):
        plan(d)["entryPoints"].append({"task": "U", "state": "A"})

    assert codes(variant(edit), "error") == ["entry-point"]


def test_key_mapping_to_missing_child_key():
    def edit(d):
        plan(d)["states"]["A"] = [{"B": {"in": {"nope": "v"}}}]

    assert codes(variant(edit), "error") == ["key-missing"]


def test_key_mapping_from_missing_parent_key():
    def edit(d):
        plan(d)["states"]["A"] = [{"B": {"in": {"speed": "nope"}}}]

    assert codes(variant(edit), "error") == ["key-missing"]


def test_key_mapping_type_mismatch():
    def edit(d):
        plan(d)["states"]["A"] = [{"B": {"in": {"speed": "n"}, "out": {"v": "done"}}}]

    findings = variant(edit)
    assert codes(findings, "error") == ["key-type"]
    assert len([f for f in findings if f.code == "key-type"]) == 2


def test_key_mapping_into_std_any_is_fine():
    def edit(d):
        plan(d)["states"]["A"] = [{"B": {"in": {"anything": "n", "speed": "v"}, "out": {"ok": "done"}}}]

    assert variant(edit) == []


def test_constant_mapping_checks_the_type():
    def edit(d):
        plan(d)["states"]["A"] = [{"B": {"in": {"speed": {"value": "fast"}, "anything": {"value": 1}}}}]

    findings = variant(edit)
    assert codes(findings, "error") == ["key-type"]
    assert len(findings) == 2


def test_transition_condition_mapping_is_checked():
    def edit(d):
        plan(d)["transitions"] = [{"A -> success": {"condition": "Keyed", "in": {"flag": "ok", "count": "v"}}}]

    findings = variant(edit)
    assert codes(findings, "error") == ["key-type"]
    assert "Keyed.count" in findings[0].message


def test_unsupported_blackboard_type():
    def edit(d):
        plan(d)["blackboard"]["pose"] = "geometry_msgs::Pose"

    assert codes(variant(edit), "error") == ["blackboard"]


def test_synchronisation_needs_two_transitions():
    def edit(d):
        plan(d)["synchronisations"] = {"s": {"transitions": ["A -> success"]}}

    assert codes(variant(edit)) == ["synchronisation"]


def test_raw_reference_that_does_not_resolve():
    def edit(d):
        plan(d)["states"]["A"] = ["Gone.beh#42"]

    assert codes(variant(edit), "error") == ["ref-unresolved"]


def test_raw_reference_with_wrong_file_name():
    data = copy.deepcopy(BASE)
    docs = compile_modules((data, None))
    beh = next(d for d in docs if d.kind == "behaviour")
    plan_doc = next(d for d in docs if d.kind == "plan")
    plan_doc.data["states"][0]["confAbstractPlanWrappers"][0]["abstractPlan"] = f"Other.beh#{beh.data['id']}"
    assert codes(check_docs(docs), "error") == ["ref-wrong-file"]


def test_raw_reference_to_wrong_kind():
    data = copy.deepcopy(BASE)
    docs = compile_modules((data, None))
    tsk = next(d for d in docs if d.kind == "taskrepository")
    plan_doc = next(d for d in docs if d.kind == "plan")
    plan_doc.data["states"][0]["confAbstractPlanWrappers"][0]["abstractPlan"] = f"TaskRepository.tsk#{tsk.data['tasks'][0]['id']}"
    assert codes(check_docs(docs), "error") == ["ref-wrong-kind"]


def test_broken_back_references_in_json():
    docs = compile_modules((copy.deepcopy(BASE), None))
    plan_doc = next(d for d in docs if d.kind == "plan")
    plan_doc.data["states"][0]["outTransitions"] = []
    plan_doc.data["states"][1]["parentPlan"] = 1
    found = [f for f in check_docs(docs) if f.code == "back-ref"]
    assert len(found) == 2


def test_duplicate_ids():
    def edit(d):
        plan(d)["states"]["A"] = {"id": 77, "children": ["B"]}
        plan(d)["states"]["success"] = {"id": 77}

    assert "duplicate-id" in codes(variant(edit), "error")


def test_duplicate_file_names_in_two_folders(tmp_path):
    for sub in ("a", "b"):
        (tmp_path / sub).mkdir()
        (tmp_path / sub / "X.beh").write_text('{"id": %d, "name": "X", "blackboard": []}' % (1 if sub == "a" else 2))
    assert codes(check_docs(load_dirs([tmp_path])), "error") == ["duplicate-file"]


def test_invalid_json_is_reported(tmp_path):
    (tmp_path / "Bad.pml").write_text("{not json")
    assert codes(check_docs(load_dirs([tmp_path])), "error") == ["parse"]


@pytest.mark.parametrize(
    "edit, message",
    [
        (lambda d: plan(d).update(colour="red"), "unknown key 'colour'"),
        (lambda d: plan(d)["states"].update(A=["Nope"]), "no plan or behaviour or plantype or placeholder named 'Nope'"),
        (lambda d: plan(d).update(transitions=[{"A -> Nowhere": "C"}]), "'A -> Nowhere' must be 'From -> To'"),
        (lambda d: plan(d).update(transitions=[{"A -> success": "NoCond"}]), "no condition named 'NoCond'"),
        (lambda d: plan(d).update(transitions=[{"A -> success": None}]), "transition needs a condition"),
        (lambda d: plan(d)["entryPoints"][0].update(task="Nope"), "no task named 'Nope'"),
        (lambda d: plan(d)["entryPoints"][0].update(state="Nope"), "initial state 'Nope' is not a state"),
        (lambda d: plan(d)["states"].update(failure={"terminal": "success"}), "named like a failure state"),
        (lambda d: plan(d)["states"].update(success={"children": ["B"]}), "terminal states cannot run"),
        (lambda d: plan(d)["states"]["A"].__setitem__(0, {"B": {"in": {"speed": 3}}}), "must be a parent key or {value: ...}"),
    ],
)
def test_compile_errors(edit, message):
    data = copy.deepcopy(BASE)
    edit(data)
    with pytest.raises(CompileError) as info:
        compile_modules((data, None))
    assert message in str(info.value)


def test_ambiguous_reference_needs_an_extension():
    data = copy.deepcopy(BASE)
    data["plans"]["B"] = {"entryPoints": [{"task": "T", "state": "S"}], "states": {"S": None}}
    with pytest.raises(CompileError) as info:
        compile_modules((data, None))
    assert "'B' is ambiguous; write one of B.pml, B.beh" in str(info.value)
    plan(data)["states"]["A"] = ["B.beh"]
    compile_modules((data, None))


def test_named_and_unnamed_terminal_states():
    data = copy.deepcopy(BASE)
    plan(data)["states"].update({"Scored": {"terminal": "success"}, "failure": None, "failure.2": None})
    plan(data)["transitions"] += [{"A -> Scored": "C"}, {"A -> failure": "C"}, {"A -> failure.2": "C"}]
    docs = compile_modules((data, None))
    states = next(d for d in docs if d.kind == "plan").data["states"]
    assert [(s["name"], s["type"], s.get("success")) for s in states] == [
        ("A", "State", None),
        ("Success", "TerminalState", True),
        ("Scored", "TerminalState", True),
        ("Failure", "TerminalState", False),
        ("Failure2", "TerminalState", False),
    ]


def test_quantifiers_get_ids_and_named_scopes():
    data = copy.deepcopy(BASE)
    plan(data)["states"]["plan"] = ["B"]
    plan(data)["transitions"].append({"A -> plan": "C"})
    plan(data)["pre"] = {
        "quantifiers": [
            {"scope": "plan", "sorts": ["X"]},
            {"scope": "state:plan", "sorts": ["Y"]},
            {"scope": "ep:T", "sorts": ["Z"]},
        ]
    }
    docs = compile_modules((data, None))
    assert check_docs(docs) == []
    p = next(d for d in docs if d.kind == "plan").data
    state_id = next(s["id"] for s in p["states"] if s["name"] == "plan")
    quants = p["preCondition"]["quantifiers"]
    assert [q["scope"] for q in quants] == [p["id"], state_id, p["entryPoints"][0]["id"]]
    assert all(q["quantifierType"] == "all" and isinstance(q["id"], int) for q in quants)
    assert len({q["id"] for q in quants}) == 3


def test_unknown_quantifier_scope_is_a_compile_error():
    data = copy.deepcopy(BASE)
    plan(data)["pre"] = {"quantifiers": [{"scope": "A", "sorts": []}]}
    with pytest.raises(CompileError) as info:
        compile_modules((data, None))
    assert "'state:<name>'" in str(info.value)
