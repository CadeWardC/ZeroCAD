"""Reject warnings beyond the reviewed backlog in both Cargo workspaces.

Use --record only after reviewing diagnostics. Identity ignores line movement,
but includes workspace, source file, lint code, message, and occurrence count.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "scripts" / "clippy-baseline.json"


def collect():
    warnings = Counter()
    for workspace, args in [(".", ["-p", "zerocad-core"]), ("OpenRCAD", ["--workspace"])]:
        process = subprocess.Popen(
            ["cargo", "clippy", *args, "--all-targets", "--locked", "--message-format=json"],
            cwd=ROOT / workspace, stdout=subprocess.PIPE, text=True, encoding="utf-8",
        )
        for line in process.stdout:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get("reason") != "compiler-message":
                continue
            message = event["message"]
            if message["level"] == "error":
                print(message.get("rendered", message["message"]), file=sys.stderr)
            if message["level"] != "warning":
                continue
            span = next((s for s in message["spans"] if s["is_primary"]), {})
            source = span.get("file_name", "<no source>").replace("\\", "/")
            code = (message.get("code") or {}).get("code", "unknown")
            key = json.dumps([workspace, source, code, message["message"]], ensure_ascii=True)
            warnings[key] += 1
        if process.wait():
            raise RuntimeError(f"Clippy failed in {workspace}")
    return warnings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", action="store_true")
    args = parser.parse_args()
    actual = collect()
    if args.record:
        BASELINE.write_text(json.dumps(dict(sorted(actual.items())), indent=2) + "\n", encoding="utf-8")
        print(f"Recorded {sum(actual.values())} warning occurrences; review the baseline diff.")
        return
    expected = Counter(json.loads(BASELINE.read_text(encoding="utf-8")))
    added = actual - expected
    if added:
        for key, count in sorted(added.items()):
            print(f"New warning ({count}): {key}", file=sys.stderr)
        raise SystemExit(1)
    print(f"Clippy passed: no warnings beyond the reviewed baseline ({sum(actual.values())} occurrences).")


if __name__ == "__main__":
    main()
