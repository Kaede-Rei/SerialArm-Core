#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BINDINGS = (ROOT / "python/bindings.cpp").read_text(encoding="utf-8")

needle = 'module.def("load_robot_profile_core"'
start = BINDINGS.index(needle)
end = BINDINGS.index('module.def("validate_robot_core_cfg"', start)
block = BINDINGS[start:end]

assert 'const std::vector<std::string>& resource_paths' in block
assert 'options.profile_file = profile_file;' in block
assert 'options.resource_paths = resource_paths;' in block
assert 'py::arg("profile_file") = ""' in block
assert 'py::arg("resource_paths") = std::vector<std::string>{}' in block

print("PYTHON_PROFILE_BINDING_CONTRACT_PASS")
