# Instructions for Claude Code

## Commits and pull requests

- Do **not** add an AI co-author footer (e.g. `Co-Authored-By: Claude ...`) to git commit messages.
- Do **not** add a "Generated with [Claude Code]" or any other AI-agent-attribution footer to pull request descriptions.

## Documentation

Project architecture lives in `docs/architecture/`, split by how often each piece changes:

- `ARCHITECTURE.md` — current-state facts only. Describes decisions already made and merged, never a forecast of future work. Updated in place as the system changes.
- `decisions/NNNN-title.md` — Architecture Decision Records (ADRs). Point-in-time reasoning for one specific choice (Title / Status / Context / Decision / Consequences / Alternatives considered). Immutable once `Accepted` — a changed mind gets a new numbered ADR that supersedes the old one, never an edit to it.
- `modules/<module>.md` — one file per real module (HAL driver, control algorithm, etc.), added in the same PR that introduces that module's first working code. Never written ahead of the code it describes.

Two rules that follow from this:

- **Don't treat GitHub issues (roadmaps, plans, proposals) as verified specs.** They're drafts until the team has actually reviewed and acted on them — design against `docs/architecture/`, not an issue body, however detailed it looks.
- **Avoid Big Design Up Front (BDUF).** Don't write architecture or module docs for work that hasn't been built yet — that's how docs and reality quietly drift apart. Just Enough Design Up Front (JEDUF): document what's real, at the point it becomes real.

## Hardware abstraction policy

- HAL/driver modules (motors, encoders, ultrasonic, line sensors, and similar timing- or performance-critical peripherals) access hardware via direct AVR registers — never `digitalWrite`/`digitalRead`/`analogRead`/`attachInterrupt`.
- Non-critical glue code (Serial debug/telemetry, status LEDs/buzzer, one-shot boot-time reads of switches/pots, vetted libraries such as an MPU-6050 driver over `Wire.h`) may use the Arduino framework.
- Full reasoning: `docs/architecture/decisions/0001-hybrid-hardware-abstraction.md`.
