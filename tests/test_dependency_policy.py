"""Dependency-policy checks; no CARLA/Cyber service is started."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


def setup_value(name):
    tree = ast.parse((ROOT / "setup.py").read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if node.func.attr == "setup":
                return ast.literal_eval(next(k.value for k in node.keywords if k.arg == name))
    raise AssertionError("setuptools.setup not found")


def permits_stable(requirement, version):
    # This policy deliberately uses only >= and != stable-release clauses.
    # The actual built distribution is also checked with a PEP 440 resolver.
    candidate = tuple(int(part) for part in version.split("."))
    for clause in requirement[len("protobuf"):].split(","):
        expected = tuple(int(part) for part in clause[2:].split("."))
        if clause[:2] == ">=" and candidate < expected:
            return False
        if clause[:2] == "!=" and candidate == expected:
            return False
        if clause[:2] not in (">=", "!="):
            raise AssertionError("unexpected policy clause: " + clause)
    return True


class DependencyPolicyTest(unittest.TestCase):
    def test_preserves_python_36_contract(self):
        self.assertEqual(setup_value("python_requires"), ">=3.6")

    def test_excludes_known_2022_vulnerable_releases(self):
        requirement, = setup_value("install_requires")
        for version in ("3.17.0", "3.17.3", "3.18.2", "3.19.4", "3.20.0", "3.20.1",
                        "4.21.0", "4.21.1", "4.21.2", "4.21.3", "4.21.4", "4.21.5"):
            with self.subTest(version=version):
                self.assertFalse(permits_stable(requirement, version))

    def test_allows_legacy_floor_and_patched_newer_lines(self):
        requirement, = setup_value("install_requires")
        for version in ("3.19.6", "3.20.2", "3.20.3", "4.21.6", "4.25.8", "5.29.6", "6.33.5"):
            with self.subTest(version=version):
                self.assertTrue(permits_stable(requirement, version))


if __name__ == "__main__":
    unittest.main()
