# Clang-Format Setup Variants

This repo uses `.clang-format` (Google base style) and `.clang-format-ignore` to keep formatting consistent.

## Repository Files

- `.clang-format`: Google style base config.
- `.clang-format-ignore`: Ignore non-code directories (`.git/`, `docs/`).

## System Setup Variants (Windows)

1. LLVM installer (recommended)

- Install LLVM from https://github.com/llvm/llvm-project/releases
- Ensure `clang-format.exe` is on PATH (usually `C:\Program Files\LLVM\bin`).
- Verify: `clang-format --version`

2. STM32CubeIDE / Other Tools

- Many embedded toolchains include `clang-format.exe`.
- For STM32CubeIDE, it is often located in:
  `C:\ST\STM32CubeIDE_1.x.x\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32...`
- You **must** add the directory containing `clang-format.exe` to your system **PATH**.
- Verify with: `where.exe clang-format`

3. Package manager

- Example: `choco install llvm`

## VS Code Setup

The repository provides recommended settings in `.vscode/settings.json` for each project. To use them:

1. Install extension: `ms-vscode.cpptools` (C/C++ extension)
2. The project settings will automatically use the `clang-format` found in your **PATH**.

Recommended Settings (configured in project `.vscode/settings.json`):

```json
{
  "editor.formatOnSave": true,
  "C_Cpp.clang_format_style": "file",
  "C_Cpp.formatting": "clangFormat",
  "C_Cpp.clang_format_fallbackStyle": "Google"
}
```

## CLI Usage Examples

- Format all C/C++ files using ripgrep:

```powershell
rg --files -g '*.c' -g '*.h' -g '*.cpp' -g '*.hpp' | ForEach-Object { clang-format -i $_ }
```

Notes:

- `.clang-format-ignore` is honored by clang-format (v14+). Keep it updated if you add non-code folders.
- Run formatting commands from the repo root so relative paths resolve correctly.
