"""Move the SO-101 KOMPAS workspace into stable folders and rewrite Sources paths.

The input workspace is snapshotted separately before this script is used.  The
script only changes paths inside `Sources` ZIP entries; geometry is copied
verbatim.  Run it from the repository root.
"""

from __future__ import annotations

import os
import re
import shutil
import tempfile
import zipfile
from pathlib import Path


ROOT = Path("arm100/robot/03_kompas_workspace").resolve()


def category(name: str) -> Path:
    n = name.casefold()
    if n == "so101 assembly.a3d":
        return Path("00_assembly")
    if any(x in n for x in ("under_arm", "upper_arm")):
        return Path("10_mechanical/arm_links")
    if any(x in n for x in ("wrist", "rotation_pitch", "motor_holder_so101_wrist")):
        return Path("10_mechanical/wrist")
    if "moving_jaw" in n:
        return Path("10_mechanical/gripper")
    if "wiring_holder" in n:
        return Path("10_mechanical/cable_management")
    if any(x in n for x in ("base", "component", "050375033")):
        return Path("10_mechanical/base_and_structure")
    if any(x in n for x in ("hexagon", "pan head", "tapping screw", "spacer")):
        return Path("20_hardware/fasteners")
    if any(x in n for x in ("scs215", "st3215", "servo v2", "motor-1723", "ge_27", "sg-ziji", "xg-ziji", "zk_122", "金属舵盘")):
        return Path("20_hardware/servos")
    if any(x in n for x in (
        "bus servo", "board_", "pcb-", "user_library", "c_0603", "c1_", "c2_", "c3_", "c4_", "c5_", "c6_", "c7_", "c8_", "c9_", "c10_",
        "d1_", "d2_", "d3_", "d4_", "dc1_", "h1_", "h2_", "h3_", "h4_", "h5_", "led1_", "m1_", "q1_", "r0603", "r1_", "r2_", "r3_", "r4_", "r5_", "r6_", "r7_", "r8_", "r9_", "r_0603", "to-223", "type_c1", "u1_", "u2_", "u3_", "u4_",
        "sod323", "sot-23", "dc-005", "extruded", "solid", "165319", "165324", "698b", "736125", "840721", "5_1_", "3_1_", "2_1_", "_1_1_", "_2_1_", "_7_1_", "x2564",
    )):
        return Path("30_electronics/bus_servo_adapter")
    return Path("99_unclassified")


def decode_utf16(data: bytes) -> tuple[str, str]:
    if data.startswith(b"\xfe\xff"):
        return data[2:].decode("utf-16-be"), "be"
    if data.startswith(b"\xff\xfe"):
        return data[2:].decode("utf-16-le"), "le"
    return data.decode("utf-16"), "native"


def encode_utf16(text: str, endian: str) -> bytes:
    if endian == "be":
        return b"\xfe\xff" + text.encode("utf-16-be")
    if endian == "le":
        return b"\xff\xfe" + text.encode("utf-16-le")
    return text.encode("utf-16")


def rewrite_sources(path: Path, destinations: dict[str, Path]) -> list[str]:
    unresolved: list[str] = []
    with zipfile.ZipFile(path, "r") as zin:
        entries = zin.infolist()
        content = {entry.filename: zin.read(entry.filename) for entry in entries}
    if "Sources" not in content:
        return unresolved

    xml, endian = decode_utf16(content["Sources"])

    def replace(match: re.Match[str]) -> str:
        original = match.group(1)
        filename = original.replace("\\", "/").rsplit("/", 1)[-1]
        target = destinations.get(filename)
        if target is None:
            unresolved.append(original)
            return match.group(0)
        relative = os.path.relpath(target, path.parent).replace("/", "\\")
        return f"<path>{relative}</path>"

    rewritten = re.sub(r"<path>(.*?)</path>", replace, xml)
    content["Sources"] = encode_utf16(rewritten, endian)
    with tempfile.NamedTemporaryFile(delete=False, suffix=path.suffix, dir=path.parent) as temp:
        temporary = Path(temp.name)
    try:
        with zipfile.ZipFile(temporary, "w") as zout:
            for entry in entries:
                zout.writestr(entry, content[entry.filename])
        shutil.move(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
    return unresolved


def main() -> None:
    files = [p for p in ROOT.iterdir() if p.is_file() and p.suffix.casefold() in {".a3d", ".m3d"}]
    destinations: dict[str, Path] = {}
    if files:
        for path in files:
            destination = ROOT / category(path.name) / path.name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(path, destination)
            destinations[path.name] = destination.resolve()
    else:
        for path in ROOT.rglob("*"):
            if path.is_file() and path.suffix.casefold() in {".a3d", ".m3d"}:
                if path.name in destinations:
                    raise SystemExit(f"Duplicate KOMPAS filename prevents safe rewrite: {path.name}")
                destinations[path.name] = path.resolve()
    if not destinations:
        raise SystemExit("No KOMPAS files found in workspace.")

    unresolved: dict[Path, list[str]] = {}
    for path in destinations.values():
        if path.suffix.casefold() == ".a3d":
            missing = rewrite_sources(path, destinations)
            if missing:
                unresolved[path.relative_to(ROOT)] = missing

    invalid_archives: list[Path] = []
    broken_paths: dict[Path, list[str]] = {}
    for path in destinations.values():
        if path.suffix.casefold() != ".a3d":
            continue
        with zipfile.ZipFile(path, "r") as archive:
            if archive.testzip() is not None:
                invalid_archives.append(path.relative_to(ROOT))
            data = archive.read("Sources") if "Sources" in archive.namelist() else b""
        xml, _ = decode_utf16(data)
        for value in re.findall(r"<path>(.*?)</path>", xml):
            reference = value.replace("\\", os.sep)
            if not (path.parent / reference).exists():
                broken_paths.setdefault(path.relative_to(ROOT), []).append(value)

    report = ROOT / "LINK_REWRITE_REPORT.txt"
    lines = ["KOMPAS Sources rewrite report", f"Moved files: {len(destinations)}", ""]
    if unresolved:
        lines.append("References absent from this workspace (kept unchanged):")
        for source, names in sorted(unresolved.items()):
            lines.append(f"- {source}")
            lines.extend(f"  - {name}" for name in names)
    else:
        lines.append("All Sources references resolve to files in this workspace.")
    lines.append("")
    lines.append(f"ZIP integrity failures: {len(invalid_archives)}")
    lines.append(f"Physical unresolved Sources paths: {sum(map(len, broken_paths.values()))}")
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(report)
    print(f"moved={len(destinations)} unresolved={sum(map(len, unresolved.values()))}")


if __name__ == "__main__":
    main()
