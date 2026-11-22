import subprocess

import pytest

from alicaplan.cli import main

from conftest import EXAMPLE, FIXTURES, REPO


def load(loader, master, roleset, *dirs, mapping=None):
    cmd = [loader] + (["--placeholder-mapping", str(mapping)] if mapping else []) + [master, roleset] + [str(d) for d in dirs]
    return subprocess.run(cmd, capture_output=True, text=True)


def test_engine_loads_the_example(tmp_path, loader):
    assert main(["compile", str(EXAMPLE), "-o", str(tmp_path)]) == 0
    result = load(loader, "Match", "RoleSet", tmp_path)
    assert result.returncode == 0, result.stderr
    assert "5 plans" in result.stdout


@pytest.mark.parametrize("master", ["MasterPlan", "SimpleTestPlan", "MultiAgentTestMaster", "RealMasterPlanForSyncTest", "ConstraintTestMaster"])
def test_engine_loads_test_plans_regenerated_without_ids(tmp_path, loader, master):
    main(["decompile", str(FIXTURES["alica_tests"]), "-o", str(tmp_path / "src"), "--no-ids"])
    main(["compile", str(tmp_path / "src"), "-o", str(tmp_path / "etc"), "--force", "-q"])
    result = load(loader, master, "Roleset", tmp_path / "etc")
    assert result.returncode == 0, result.stderr


def test_engine_loads_turtlesim_regenerated_without_ids(tmp_path, loader):
    main(["decompile", str(FIXTURES["alica_turtlesim"]), "-o", str(tmp_path / "src"), "--no-ids"])
    assert main(["compile", str(tmp_path / "src"), "-o", str(tmp_path / "etc"), "-q"]) == 0
    result = load(loader, "FourCornersTutorial", "Roleset", tmp_path / "etc")
    assert result.returncode == 0, result.stderr


def test_engine_rejects_a_missing_master_plan(tmp_path, loader):
    main(["compile", str(EXAMPLE), "-o", str(tmp_path)])
    assert load(loader, "NoSuchPlan", "RoleSet", tmp_path).returncode != 0


def test_original_fixtures_load(loader):
    etc = FIXTURES["alica_tests"]
    result = load(loader, "TestMasterPlan", "Roleset", etc, mapping=REPO / "alica_tests" / "config" / "placeholder_mapping.json")
    assert result.returncode == 0, result.stderr
