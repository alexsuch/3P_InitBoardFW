#!/usr/bin/env python3
"""
Prebuild script to scan component assets folder and generate C arrays compiled into firmware.

This script scans main/assets/ for files and generates C arrays with embedded binary data.
This approach is more reliable than ESP-IDF's EMBED_FILES mechanism which can have linking issues.
"""
import os
import re
from pathlib import Path


def _write_text_if_changed(path: Path, content: str) -> bool:
    """
    Write file only if contents differ.

    Returns True if the file was written/updated.
    """
    try:
        existing = path.read_text(encoding="utf-8")
        if existing == content:
            return False
    except FileNotFoundError:
        pass
    path.write_text(content, encoding="utf-8", newline="\n")
    return True


def _get_project_root() -> Path:
    """
    Resolve PlatformIO project root.

    - When running directly: __file__ exists.
    - When running as PlatformIO extra_script: __file__ may be missing; cwd is typically project root.
      Prefer env vars if present.
    """
    # PlatformIO commonly provides this env var
    pio_root = os.environ.get("PLATFORMIO_PROJECT_DIR") or os.environ.get("PROJECT_DIR")
    if pio_root:
        return Path(pio_root)
    try:
        # Script is in web-ui/scripts/, project root is parent of web-ui (i.e., logger/)
        return Path(__file__).resolve().parent.parent.parent
    except NameError:
        return Path(os.getcwd())


PROJECT_ROOT = _get_project_root()
COMPONENT_DIR = PROJECT_ROOT / "main"
ASSETS_DIR = COMPONENT_DIR / "assets"
CMAKE_LISTS = COMPONENT_DIR / "CMakeLists.txt"
PLATFORMIO_INI = PROJECT_ROOT / "platformio.ini"


def scan_data_files():
    """
    Scan component assets folder and return embed paths relative to component directory.

    Expected layout:
      <project>/main/assets/**
    
    Paths are relative to main/ component directory:
      - main/assets/index.html.gz → "assets/index.html.gz"
      - main/assets/assets/file.js.gz → "assets/assets/file.js.gz"
    """
    if not ASSETS_DIR.exists():
        print(f"Warning: {ASSETS_DIR} does not exist")
        return []

    embedded_rel_paths: list[str] = []
    for p in sorted(ASSETS_DIR.rglob("*")):
        if not p.is_file():
            continue
        # Paths relative to component directory (main/)
        rel_to_component = p.relative_to(COMPONENT_DIR)
        embedded_rel_paths.append(str(rel_to_component).replace("\\", "/"))
    return embedded_rel_paths


def update_cmake_lists(embed_files: list[str]) -> bool:
    """
    Remove any existing EMBED_FILES from CMakeLists.txt (we embed via generated C arrays now).
    """
    if not CMAKE_LISTS.exists():
        print(f"Error: {CMAKE_LISTS} does not exist")
        return False
    
    # Read current content
    with open(CMAKE_LISTS, "r", encoding="utf-8", newline="") as f:
        lines = f.readlines()
    
    # Check if idf_component_register exists
    content_str = "".join(lines)
    if "idf_component_register" not in content_str:
        print(f"Error: idf_component_register not found in {CMAKE_LISTS}")
        return False
    
    # Remove any existing EMBED_FILES lines
    embed_line_re = re.compile(r'^\s*EMBED_FILES\s+.*$')
    new_lines = [ln for ln in lines if not embed_line_re.match(ln)]
    
    # Only rewrite if something changed
    new_content = ''.join(new_lines)
    if new_content != content_str:
        with open(CMAKE_LISTS, "w", encoding="utf-8", newline="") as f:
            f.write(new_content)
        return True
    
    return True




def update_platformio_ini(embed_files: list[str]) -> bool:
    """
    Update platformio.ini to include board_build.embed_files with all asset paths.
    
    Paths in platformio.ini are relative to project root (with main/ prefix).
    Removes any existing board_build.embed_files section and adds new one.
    """
    if not PLATFORMIO_INI.exists():
        # File doesn't exist, that's okay - skip silently
        return True
    
    # Read current content
    with open(PLATFORMIO_INI, "r", encoding="utf-8", newline="") as f:
        lines = f.readlines()
    
    # Generate paths relative to project root (with main/ prefix)
    project_root_paths = [f"main/{path}" for path in embed_files]
    
    # Find and replace board_build.embed_files section
    result_lines = []
    i = 0
    embed_section_found = False
    
    while i < len(lines):
        line = lines[i]
        
        # Check if this is the start of board_build.embed_files
        if re.match(r'^\s*board_build\.embed_files\s*=', line):
            embed_section_found = True
            result_lines.append(line)
            i += 1
            
            # Skip all continuation lines (indented lines following the =)
            # Also skip empty lines that are between continuation lines
            while i < len(lines):
                next_line = lines[i]
                stripped = next_line.lstrip()
                
                # If it's an indented line with content, it's a continuation
                if stripped and len(next_line) - len(stripped) > 0:
                    i += 1
                    continue
                # If it's an empty line, check what comes after
                elif not stripped:
                    # Look ahead to see if there's another continuation line
                    j = i + 1
                    while j < len(lines) and not lines[j].strip():
                        j += 1
                    # If next non-empty line is indented, skip this empty line too
                    if j < len(lines):
                        next_non_empty = lines[j]
                        if next_non_empty.strip() and len(next_non_empty) - len(next_non_empty.lstrip()) > 0:
                            i += 1
                            continue
                    # Otherwise, we've reached the end of the section
                    break
                else:
                    # Non-indented line with content - end of section
                    break
            
            # Add new embed_files entries
            if project_root_paths:
                indent = '  '  # Standard 2-space indent for continuation
                for path in project_root_paths:
                    result_lines.append(f"{indent}{path}\n")
            
            # Continue processing from current line (don't skip it)
            continue
        else:
            result_lines.append(line)
            i += 1
    
    # Only rewrite if something changed
    new_content = ''.join(result_lines)
    original_content = ''.join(lines)
    
    if new_content != original_content:
        with open(PLATFORMIO_INI, "w", encoding="utf-8", newline="") as f:
            f.write(new_content)
        return True
    
    return True


def _c_identifier_from_path(rel_path: str) -> str:
    """
    Convert a file path to a valid C identifier.
    
    Example: assets/index.html.gz -> assets_index_html_gz
    """
    # Replace non-alphanumeric characters with underscores
    ident = re.sub(r"[^a-zA-Z0-9_]", "_", rel_path)
    # Collapse multiple underscores
    ident = re.sub(r"_+", "_", ident).strip("_")
    if not ident:
        ident = "asset"
    # Ensure it doesn't start with a digit
    if ident[0].isdigit():
        ident = "asset_" + ident
    return ident.lower()


def generate_assets_header(embed_files: list[str]) -> bool:
    """
    Generate main/generated_assets.h.
    """
    header_path = COMPONENT_DIR / "generated_assets.h"
    
    lines = []
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <stdint.h>")
    lines.append("#include <stddef.h>")
    lines.append("")
    lines.append("// Auto-generated by update-embed-files.py")
    lines.append("")
    lines.append("typedef struct {")
    lines.append("    const char* path;")
    lines.append("    const uint8_t* start;")
    lines.append("    const uint8_t* end;")
    lines.append("    const char* mime_type;")
    lines.append("    int is_gzipped;")
    lines.append("} embedded_asset_t;")
    lines.append("")
    lines.append("const embedded_asset_t* embedded_asset_find(const char* path);")
    lines.append("")
    
    return _write_text_if_changed(header_path, "\n".join(lines))


def generate_assets_source(embed_files: list[str]) -> bool:
    """
    Generate main/generated_assets.c with embedded binary data as C arrays.
    """
    source_path = COMPONENT_DIR / "generated_assets.c"
    
    # Read file data for each asset
    blobs: list[tuple[str, str, bytes]] = []
    for rel_path in embed_files:
        # Paths are relative to component directory (main/)
        abs_path = COMPONENT_DIR / rel_path
        try:
            data = abs_path.read_bytes()
        except FileNotFoundError:
            # If assets were removed mid-build, just skip
            print(f"Warning: File not found: {abs_path}")
            continue
        sym = _c_identifier_from_path(rel_path)
        blobs.append((rel_path, sym, data))
    
    lines = []
    lines.append("// Auto-generated by scripts/update-embed-files.py. Do not edit.")
    lines.append('#include "generated_assets.h"')
    lines.append('#include <string.h>')
    lines.append("")
    
    # Generate C arrays for each asset
    for rel_path, sym, data in blobs:
        lines.append(f"// {rel_path}")
        lines.append(f"static const uint8_t {sym}[] = {{")
        # 12 bytes per line for readability
        for i in range(0, len(data), 12):
            chunk = data[i : i + 12]
            lines.append("    " + ", ".join(f"0x{b:02x}" for b in chunk) + ",")
        lines.append("};")
        lines.append("")
    
    # Generate lookup table
    lines.append("static const embedded_asset_t g_assets[] = {")
    
    for rel_path, sym, data in blobs:
        # Check for gzip
        is_gz = 1 if rel_path.endswith(".gz") else 0
        
        # Web path: remove "assets/" prefix if present
        web_path = rel_path
        if web_path.replace("\\", "/").startswith("assets/"):
            web_path = web_path[7:]  # len("assets/")
        
        lines.append("    {")
        lines.append(f'        .path = "{web_path}",')
        lines.append(f"        .start = {sym},")
        lines.append(f"        .end = {sym} + {len(data)},")
        lines.append(f'        .mime_type = NULL,')
        lines.append(f'        .is_gzipped = {is_gz},')
        lines.append("    },")
    
    lines.append("    {NULL, NULL, NULL, NULL, 0} // Sentinel")
    lines.append("};")
    lines.append("")
    
    # Generate lookup function
    lines.append("const embedded_asset_t* embedded_asset_find(const char* path) {")
    lines.append("    if (!path) return NULL;")
    lines.append("    for (const embedded_asset_t* p = g_assets; p->path != NULL; ++p) {")
    lines.append("        if (strcmp(p->path, path) == 0) {")
    lines.append("            return p;")
    lines.append("        }")
    lines.append("    }")
    lines.append("    return NULL;")
    lines.append("}")
    lines.append("")
    
    return _write_text_if_changed(source_path, "\n".join(lines))


def main():
    print(f"Scanning assets folder for files to embed: {ASSETS_DIR}")
    embed_files = scan_data_files()
    
    if embed_files:
        print(f"Found {len(embed_files)} file(s) to embed:")
        for file in embed_files:
            print(f"  - {file}")
    else:
        print("No files found in assets folder")
    
    print(f"Updating {CMAKE_LISTS}...")
    if update_cmake_lists(embed_files):
        print("Successfully updated CMakeLists.txt")
    else:
        print("Failed to update CMakeLists.txt")
        return 1
    
    # Update platformio.ini if it exists
    if PLATFORMIO_INI.exists():
        print(f"Updating {PLATFORMIO_INI}...")
        if update_platformio_ini(embed_files):
            print("Successfully updated platformio.ini")
        else:
            print("Failed to update platformio.ini")
            return 1
            
    # Generate C/H files (always generate, so build is consistent with current assets folder)
    print("Generating assets C/H files...")
    generate_assets_header(embed_files)
    source_updated = generate_assets_source(embed_files)
    if source_updated:
        print("Successfully generated assets source files")
    else:
        print("Assets source files up to date")
    
    return 0


if __name__ == "__main__":
    exit(main())
else:
    # PlatformIO `extra_scripts` are executed by SCons via import/exec (not as __main__),
    # so we must run main() at import time.
    try:
        Import("env")  # type: ignore[name-defined]
        # If Import("env") succeeds, we're being imported by PlatformIO
        env_imported = True
    except Exception:
        # Not being imported by PlatformIO, or Import function not available
        env_imported = False
    
    # Run main() if:
    # 1. PlatformIO imported us (Import("env") succeeded), OR
    # 2. Environment variables are set (fallback for direct execution contexts)
    if env_imported or os.environ.get("PLATFORMIO_PROJECT_DIR") or os.environ.get("PROJECT_DIR"):
        rc = main()
        if rc != 0:
            raise SystemExit(rc)

