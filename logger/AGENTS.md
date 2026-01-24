# Repository Guidelines

## Project Structure & Modules
- Firmware sources live in `main/` (core app logic, platform integration, IO, Wi-Fi, protocols, utilities). The project supports dual build with both PlatformIO and ESP-IDF.
- Shared libraries are in `lib/` and `components/` (e.g., HAL, sensors, HTTP server, mavlink).
- Build and configuration files live in `platformio.ini`, `sdkconfig.*`, and `Kconfig.projbuild`.
- Tests reside in `test/` and use PlatformIO's Unity-based test runner.
- Additional docs and board notes are under `doc/` (e.g., `doc/esp-hints.md`, `configuration.example.ini`, pinouts).

## Build, Test & Run
- Build for generic ESP32 dev board: `pio run -e esp32`
- Flash firmware: `pio run -e esp32 -t upload`
- Open serial monitor: `pio device monitor`
- Run unit tests for a given environment: `pio test -e esp32`

## Coding Style & Naming
- C/C++ code is formatted with `.clang-format` (Google style, 4-space indent, 160-column limit, no tabs). Run `clang-format` before committing.
- Prefer descriptive, lowercase `snake_case` for functions/variables and `UPPER_SNAKE_CASE` for macros and configuration flags.
- Group related code under existing module folders (`main/io`, `main/modules`, `main/platform`, `main/protocols`, `main/util`, `main/wifi`).
- Keep public APIs in headers (`*.h`) minimal and stable; put internal helpers in `static` functions within `*.c`/`*.cpp`.
- Write all code comments and documentation files in English only.

## Testing Guidelines
- New features should include or update tests under `test/` when practical.
- Name test files `test_<feature>.cpp` and keep each `RUN_TEST(...)` focused on one behavior.
- Use Unity assertions (e.g., `TEST_ASSERT_EQUAL`) and log output sparingly for debugging.

## Commits & Pull Requests
- Write concise, imperative commit messages (e.g., `fix wifi reconnect logic`, `add async config tests`).
- Keep changes logically scoped and avoid mixing refactors with functional changes.
- Pull requests should include: a short summary, rationale, manual testing steps/commands, and links to any related issues.
- When changes affect hardware behavior or configuration, mention the target board and attach logs or screenshots where helpful.

## Agent-Specific Instructions
- Do not run build or test commands (`pio run`, `pio test`, `cmake`, etc.); builds and tests are executed manually by maintainers.
- When external library documentation is needed, use the MCP Context7 tools (`mcp__context7__resolve-library-id` and `mcp__context7__get-library-docs`) instead of external web search.
