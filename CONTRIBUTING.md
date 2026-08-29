# Contributing to Serial Studio

Thanks for taking the time to contribute. Bug reports, code, examples, and documentation
fixes are all welcome.

## Reporting bugs

Open an issue with the bug report template. The report is most useful when it includes:

- Operating system and version
- Serial Studio version and edition (GPL build, Trial, or Pro)
- Connection type (UART, TCP/UDP, BLE, MQTT, Modbus, CAN Bus, USB, HID, Audio, Process)
- Steps to reproduce, what you expected, and what happened instead
- The project file (`.ssproj`) and console output when relevant

## Suggesting features

Open an issue with the feature request template, or start a thread in
[Discussions](https://github.com/Serial-Studio/Serial-Studio/discussions) if the idea is
still taking shape. For larger changes, talk it through in an issue before writing code so
the approach is agreed on first.

## Contributing code

- Contributions to both the GPL-licensed code and the commercial (Pro) modules are
  welcome. Every source file carries an SPDX header that states its license; check it
  before you start so you know which terms apply. Contributions to Pro modules are
  accepted under the Contributor License Agreement described below.
- Follow the project's clang-format config (LLVM base style, 100 columns, 2-space indent).
- Run `scripts/code-verify.py --check` before opening a pull request; it enforces the
  structural and style rules that CI checks. `scripts/sanitize-commit.py` runs the full
  pipeline (formatting, verification, documentation checks) in one step.
- Style details live in [doc/claude/code-style.md](doc/claude/code-style.md); repo-wide
  rules live in [CLAUDE.md](CLAUDE.md).
- Add Doxygen comments for new public APIs. Avoid inline end-of-line comments.

### Submitting changes

1. Fork the repository and create a feature branch (`git checkout -b feature/my-change`).
2. Commit with descriptive messages.
3. Push to your fork and open a pull request using the template.
4. Make sure CI passes, including the hotpath benchmark gate for changes near the data
   pipeline.

## Contributor License Agreement (CLA)

By submitting a contribution to this repository, you agree to the following terms.

You certify that:

- The contribution is your original work, or you have the legal right to submit it.
- You are legally entitled to grant the rights described below.
- If you are submitting the contribution on behalf of an entity (for example, your
  employer), you have that entity's permission to do so.

You grant Alex Spataru a perpetual, worldwide, royalty-free, irrevocable, non-exclusive
license to:

- Use, reproduce, modify, adapt, publish, distribute, sublicense, and create derivative
  works of your contribution.
- License the contribution under the GNU General Public License v3 (GPLv3) and the Serial
  Studio Commercial License, including future versions of both licenses.

You agree that:

- Your contribution may be used as part of both open-source and commercially licensed
  software.
- No part of your contribution is subject to any patent or other intellectual property
  restriction that would prevent its commercial use or distribution.
- You will not revoke or challenge this license grant in the future.

Scope:

- "Contribution" means any original work of authorship, including any modifications or
  additions to existing content, submitted via pull request, issue, or any form of
  electronic communication intended to be included in the project.

By submitting a contribution, you acknowledge that you have read, understood, and agree to
these terms.

## Contributing documentation

Help pages live in `doc/help/` and example write-ups in `examples/`. Run
`scripts/documentation-verify.py` before submitting; it lints the Markdown that ships with
the app.

## Tests

The Python test suite lives in `tests/` (see [tests/README.md](tests/README.md)).
Integration tests drive a running instance over the TCP API and need
**Settings > Miscellaneous > Enable API Server** turned on. The JS parser unit tests in
`tests/scripts/` only need Node.js.

```bash
pip install -r tests/requirements.txt
pytest tests/scripts/ -v          # parser unit tests, no app required
pytest tests/integration/ -v      # needs a running instance with the API server enabled
```

## Questions

Use [Discussions](https://github.com/Serial-Studio/Serial-Studio/discussions) for questions
and the [help center](https://serial-studio.com/help) for usage documentation.
