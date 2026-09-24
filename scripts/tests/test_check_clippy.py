"""Negative controls for the warning-regression release gate."""
from collections import Counter
from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("check_clippy", Path(__file__).parents[1] / "check-clippy.py")
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)


class WarningGateTests(unittest.TestCase):
    def run_gate(self, baseline, actual):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "baseline.json"
            path.write_text(json.dumps(baseline), encoding="utf-8")
            output = io.StringIO()
            with patch.object(gate, "BASELINE", path), patch.object(gate, "collect", return_value=Counter(actual)), patch.object(sys, "argv", ["check-clippy.py"]), redirect_stdout(output), redirect_stderr(output):
                gate.main()

    def test_unchanged_and_reduced_backlog_pass(self):
        self.run_gate({"existing": 2}, {"existing": 2})
        self.run_gate({"existing": 2}, {"existing": 1})

    def test_new_fingerprint_fails(self):
        with self.assertRaises(SystemExit) as failure:
            self.run_gate({"existing": 2}, {"existing": 2, "introduced": 1})
        self.assertEqual(failure.exception.code, 1)

    def test_additional_occurrence_of_existing_lint_fails(self):
        with self.assertRaises(SystemExit) as failure:
            self.run_gate({"existing": 2}, {"existing": 3})
        self.assertEqual(failure.exception.code, 1)


if __name__ == "__main__":
    unittest.main()
