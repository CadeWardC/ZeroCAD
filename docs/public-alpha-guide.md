# Public alpha recovery and reporting guide

ZeroCAD's public alpha is intentionally offline. It collects no telemetry and
never uploads a model, log, crash, hardware name, or report automatically.

## Recover work

After a crash or interrupted session, restart ZeroCAD. If a recovery document is
present, the status bar announces it and **File → Recover Autosave** becomes
available. Recovery loads an ordinary validated editable document. Save it under
a normal project name to keep it permanently.

Normal saves use a synchronized temporary file and preserve the previous complete
file until replacement succeeds. Autosaves use the same writer in compact form
and run away from the interface thread.

## Export a report

Choose **File → Export Bug Report…** and select a local ZIP path. Review or remove
anything you do not want to share. The bundle contains:

- `document.zcad`: the current editable recipe;
- `manifest.json`: application version, git build, OS/architecture, renderer,
  the last evaluator error, the full unresolved-feature ID/reason map, and the
  offline/user-initiated flags;
- `session.log`: this run's local application log;
- `last-panic.txt`: the latest local panic trace, when one exists.

The command performs no network action. The user decides whether and how to send
the ZIP.

## Defect acceptance discipline

For each alpha report, reproduce against its exact build first. An accepted
kernel or persistence defect is not closed until a minimal permanent test fixture
fails before the fix and passes after it. Count accepted defects and frozen
regressions separately in the Phase 7 release evidence; the release validator
requires them to match and requires zero unresolved release blockers.

Capability-boundary diagnostics are not automatically bugs. The retained 1.0
exception ledger names the current general Move Face, Delete Face, non-analytic
offset/thicken, difficult B-spline split, and STL-repair limits along with their
owners and executable removal triggers.
