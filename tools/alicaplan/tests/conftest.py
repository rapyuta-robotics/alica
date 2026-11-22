import os
import shutil
from pathlib import Path

import pytest

from alicaplan.check import check_docs
from alicaplan.compiler import Compiler, Module

REPO = Path(__file__).resolve().parents[3]
TOOL = Path(__file__).resolve().parents[1]
FIXTURES = {
    "alica_tests": REPO / "alica_tests" / "etc",
    "alica_turtlesim": REPO / "supplementary" / "alica_turtlesim" / "etc",
}
EXAMPLE = TOOL / "examples" / "football"


def compile_modules(*modules):
    mods = [Module(f"m{i}.alica.yaml", data, layout or {}) for i, (data, layout) in enumerate(modules)]
    return Compiler(mods).compile()


def findings_for(*modules):
    return check_docs(compile_modules(*modules))


def codes(findings, severity=None):
    return sorted({f.code for f in findings if severity is None or f.severity == severity})


@pytest.fixture
def loader():
    explicit = os.environ.get("ALICA_LOAD_PLANS")
    candidates = [explicit] if explicit else [str(REPO / "install" / "bin" / "alica_load_plans"), shutil.which("alica_load_plans")]
    for path in candidates:
        if path and os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    if os.environ.get("ALICA_LOAD_PLANS_REQUIRED"):
        pytest.fail("alica_load_plans not found; build alica_engine with scripts/build.sh or set ALICA_LOAD_PLANS")
    pytest.skip("alica_load_plans not built")
