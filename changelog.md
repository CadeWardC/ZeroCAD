# Changelog

## 0.7.6-alpha.1 — 2026-09-23

- Pin release builds and warning checks to the validated Rust 1.94.0 toolchain.
- Add a Debian/Ubuntu amd64 package with the existing ZeroCAD application icon
  and application-menu launcher.
- Add 119 adversarial test functions covering joins, editing history, malformed
  imports, expression limits and material preservation.
- Repair translated joins, joined-boss counterbores, generated face selection,
  cut rollback, closed sweeps and kernel edge orientation.
- Bound expression parsing to prevent the reproduced stack-overflow crash.
- Preserve source geometry when an unsupported operation is rejected.

Validation: 522 break tests, 961 core/GUI tests and 355 kernel tests passed on
the development machine. See `break-test/review/2026-09-17-repairs.md` for
evidence and supported boundaries. This is an alpha release; these results do
not establish support for every geometry combination or constitute a full GUI
or security audit.
