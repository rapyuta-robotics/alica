import filecmp
import os

import pytest

from alicaplan.check import check_docs
from alicaplan.cli import main
from alicaplan.jsonio import load_dirs, model_files

from conftest import EXAMPLE, FIXTURES


@pytest.mark.parametrize("name", sorted(FIXTURES))
def test_decompile_compile_is_byte_identical(tmp_path, name):
    etc = FIXTURES[name]
    assert main(["decompile", str(etc), "-o", str(tmp_path / "src")]) == 0
    main(["compile", str(tmp_path / "src"), "-o", str(tmp_path / "etc"), "--force", "-q"])
    files = model_files(etc)
    assert files == model_files(tmp_path / "etc")
    match, mismatch, errors = filecmp.cmpfiles(etc, tmp_path / "etc", files, shallow=False)
    assert mismatch == [] and errors == []
    assert len(match) == len(files) > 20


@pytest.mark.parametrize("name", sorted(FIXTURES))
def test_without_ids_keeps_every_finding(tmp_path, name):
    etc = FIXTURES[name]
    assert main(["decompile", str(etc), "-o", str(tmp_path / "src"), "--no-ids"]) == 0
    main(["compile", str(tmp_path / "src"), "-o", str(tmp_path / "etc"), "--force", "-q"])
    original = [(f.file, f.code, f.message) for f in check_docs(load_dirs([etc])) if "#" not in f.where]
    regenerated = [(f.file, f.code, f.message) for f in check_docs(load_dirs([tmp_path / "etc"])) if "#" not in f.where]
    assert [c for _, c, _ in regenerated] == [c for _, c, _ in original]


def test_example_compiles_clean_and_survives_a_round_trip(tmp_path):
    assert main(["compile", str(EXAMPLE), "-o", str(tmp_path / "a")]) == 0
    assert check_docs(load_dirs([tmp_path / "a"])) == []
    assert main(["decompile", str(tmp_path / "a"), "-o", str(tmp_path / "src")]) == 0
    assert main(["compile", str(tmp_path / "src"), "-o", str(tmp_path / "b")]) == 0
    files = model_files(tmp_path / "a")
    _, mismatch, errors = filecmp.cmpfiles(tmp_path / "a", tmp_path / "b", files, shallow=False)
    assert mismatch == [] and errors == []


def test_ids_are_stable_across_compiles(tmp_path):
    main(["compile", str(EXAMPLE), "-o", str(tmp_path / "a")])
    main(["compile", str(EXAMPLE), "-o", str(tmp_path / "b")])
    files = model_files(tmp_path / "a")
    _, mismatch, _ = filecmp.cmpfiles(tmp_path / "a", tmp_path / "b", files, shallow=False)
    assert mismatch == []


def test_compile_writes_layout_positions(tmp_path):
    src = tmp_path / "src"
    src.mkdir()
    (src / "p.alica.yaml").write_text(
        "task_repository: {tasks: [T]}\n"
        "condition_repository: {conditions: {C: {}}}\n"
        "behaviours: {B: {}}\n"
        "plans:\n  P:\n    entryPoints: [{task: T, state: A}]\n    states: {A: [B], success: null}\n    transitions: [{A -> success: C}]\n"
    )
    (src / "p.layout.yaml").write_text("P:\n  states: {A: [10, 20]}\n  entryPoints: {T: [1, 2]}\n  transitions: {A -> success: [[5, 6]]}\n")
    assert main(["compile", str(src), "-o", str(tmp_path / "etc")]) == 0
    plan = load_dirs([tmp_path / "etc"])
    plan = next(d.data for d in plan if d.kind == "plan")
    assert plan["states"][0]["positionWeb"] == {"x": 10, "y": 20}
    assert plan["entryPoints"][0]["positionWeb"] == {"x": 1, "y": 2}
    assert plan["transitions"][0]["pointsWeb"] == [{"x": 5, "y": 6}]
    assert plan["states"][1]["name"] == "Success"


def test_compile_refuses_to_write_when_checks_fail(tmp_path):
    src = tmp_path / "src"
    src.mkdir()
    (src / "p.alica.yaml").write_text(
        "task_repository: {tasks: [T]}\n"
        "condition_repository: {conditions: {C: {}}}\n"
        "plans:\n  M:\n    master: true\n    entryPoints: [{task: T, state: A}]\n    states: {A: null, success: null}\n    transitions: [{A -> success: C}]\n"
    )
    assert main(["compile", str(src), "-o", str(tmp_path / "etc")]) == 1
    assert not os.path.exists(tmp_path / "etc")


def test_compile_clean_removes_stale_files(tmp_path):
    main(["compile", str(EXAMPLE), "-o", str(tmp_path)])
    (tmp_path / "Old.beh").write_text("{}")
    assert main(["compile", str(EXAMPLE), "-o", str(tmp_path), "--clean"]) == 0
    assert not (tmp_path / "Old.beh").exists()


def test_docs_example_compiles_clean(tmp_path):
    from conftest import REPO

    text = (REPO / "docs" / "alicaplan.md").read_text()
    start = text.index("```yaml", text.index("## Example")) + len("```yaml")
    src = tmp_path / "src"
    src.mkdir()
    (src / "example.alica.yaml").write_text(text[start : text.index("```", start)])
    assert main(["compile", str(src), "-o", str(tmp_path / "etc")]) == 0
    assert check_docs(load_dirs([tmp_path / "etc"])) == []


def test_compile_with_missing_or_empty_source_leaves_output_alone(tmp_path):
    main(["compile", str(EXAMPLE), "-o", str(tmp_path / "etc")])
    before = model_files(tmp_path / "etc")
    assert main(["compile", str(tmp_path / "missing"), "-o", str(tmp_path / "etc"), "--clean"]) == 2
    (tmp_path / "empty").mkdir()
    assert main(["compile", str(tmp_path / "empty"), "-o", str(tmp_path / "etc"), "--clean"]) == 1
    assert model_files(tmp_path / "etc") == before
