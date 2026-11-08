#!/usr/bin/env python3
"""Provide electron-builder with the already installed Linux Electron runtime"""

import argparse
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import zipfile


def check_software_runtime(launcher: Path, dist: Path) -> None:
    """Use the desktop's software-backend resource list before creating an archive"""
    script = "const {SOFTWARE_FILES}=require(process.argv[1]); process.stdout.write(JSON.stringify(SOFTWARE_FILES))"
    result = subprocess.run(['node', '-e', script, str(launcher / 'desktop/graphics.cjs')],
                            capture_output=True, text=True, timeout=15)
    if result.returncode:
        raise ValueError('Cannot read Electron graphics runtime requirements: ' + result.stderr.strip())
    for name in json.loads(result.stdout):
        if not (dist / name).is_file():
            raise ValueError('Electron graphics runtime is missing: ' + name + '; reinstall the GUI')
    icd = json.loads((dist / 'vk_swiftshader_icd.json').read_text(encoding='utf-8'))
    if icd.get('ICD', {}).get('library_path') != './libvk_swiftshader.so':
        raise ValueError('Electron SwiftShader ICD does not reference its bundled library')


def archive_electron(launcher: Path, output: Path) -> Path:
    launcher = launcher.resolve()
    declared = json.loads((launcher / 'package.json').read_text(encoding='utf-8'))
    installed = json.loads((launcher / 'node_modules/electron/package.json').read_text(encoding='utf-8'))
    expected = declared.get('devDependencies', {}).get('electron')
    actual = installed.get('version')
    if not isinstance(expected, str) or actual != expected:
        raise ValueError(f'Electron version mismatch: GUI requires {expected}, installed {actual}; rerun ./install.sh --gui-only --yes')

    dist = launcher / 'node_modules/electron/dist'
    executable = dist / 'electron'
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise ValueError('Installed Electron binary is missing; rerun ./install.sh --gui-only --yes')
    if executable.open('rb').read(4) != b'\x7fELF':
        raise ValueError('Installed Electron executable is not an ELF binary')
    if not (dist / 'resources').is_dir():
        raise ValueError('Electron runtime resources are missing; reinstall the GUI')
    check_software_runtime(launcher, dist)

    output.mkdir(parents=True, exist_ok=True)
    archive = output / f'electron-v{expected}-linux-x64.zip'
    temporary = output / (archive.name + '.tmp')
    count = 0
    try:
        with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=1, allowZip64=True) as target:
            for source in sorted(dist.rglob('*')):
                rel = source.relative_to(dist).as_posix()
                if source.is_symlink():
                    # Preserve symlinks and refuse links that escape the Electron runtime
                    resolved = source.resolve(strict=True)
                    if not resolved.is_relative_to(dist.resolve()):
                        raise ValueError(f'Electron runtime contains an external symlink: {rel}')
                    entry = zipfile.ZipInfo(rel)
                    entry.create_system = 3
                    entry.external_attr = (stat.S_IFLNK | 0o777) << 16
                    target.writestr(entry, os.readlink(source).encode('utf-8'))
                    count += 1
                elif source.is_file():
                    target.write(source, rel)
                    count += 1
        with zipfile.ZipFile(temporary) as target:
            if count < 2 or 'electron' not in target.namelist():
                raise ValueError('Local Electron archive is missing required files')
            if target.getinfo('electron').file_size == 0:
                raise ValueError('Local Electron executable is empty')
            corrupt = target.testzip()
            if corrupt is not None:
                raise ValueError('Local Electron archive checksum check failed for: ' + corrupt)
        temporary.replace(archive)
    finally:
        temporary.unlink(missing_ok=True)
    return archive


def main():
    parser = argparse.ArgumentParser(description='Archive an installed Electron runtime for offline packaging')
    parser.add_argument('--launcher', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        archive = archive_electron(args.launcher, args.output)
    except (ValueError, OSError, json.JSONDecodeError, zipfile.BadZipFile) as error:
        print('[SerialArm] Error: ' + str(error), file=sys.stderr)
        return 1
    print('[SerialArm] Using locally installed Electron runtime: ' + str(archive))
    return 0


if __name__ == '__main__':
    sys.exit(main())
