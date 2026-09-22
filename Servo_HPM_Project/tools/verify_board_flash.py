"""Compare a read-only flash dump to allocated flash sections of a 32-bit ELF.
Usage: python verify_board_flash.py board.bin firmware.elf --base 0x80000400
Does not connect, download, or write to a board. Padding is excluded.
"""
import argparse
import hashlib
import json
import struct
from pathlib import Path


def compare(dump, elf, base):
    if len(elf) < 52 or elf[:6] != b"\x7fELF\x01\x01":
        raise ValueError("Expected ELF32 little-endian")
    header = struct.unpack_from("<16sHHIIIIIHHHHHH", elf)
    offset, entry_size, count, strings = header[6], header[11], header[12], header[13]
    if entry_size < 40 or not count or strings >= count or offset + entry_size * count > len(elf):
        raise ValueError("Unsupported or truncated section table")
    sections = [struct.unpack_from("<IIIIIIIIII", elf, offset + i * entry_size) for i in range(count)]
    tab = sections[strings]
    names = elf[tab[4]:tab[4] + tab[5]]
    checked, matched, failures = 0, 0, []
    for name, typ, flags, address, start, size, *_ in sections:
        if not size or typ == 8 or not flags & 2 or not base <= address < base + len(dump):
            continue
        if address + size > base + len(dump) or start + size > len(elf):
            raise ValueError("Truncated allocated flash section")
        expected = elf[start:start + size]
        actual = dump[address - base:address - base + size]
        checked += size
        section_name = names[name:].split(b"\0")[0].decode("utf-8", errors="replace")
        if expected == actual:
            matched += 1
        else:
            failures.append({"section": section_name, "address": hex(address),
                             "bytes": size, "different_bytes": sum(a != b for a, b in zip(expected, actual))})
    if not checked:
        raise ValueError("No allocated flash sections covered by the dump")
    return {"match": not failures, "checked_bytes": checked, "matched_sections": matched,
            "mismatched_sections": failures,
            "scope": "Allocated non-NOBITS sections in the supplied flash interval; excludes padding and runtime RAM",
            "dump_sha256": hashlib.sha256(dump).hexdigest(), "elf_sha256": hashlib.sha256(elf).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump", type=Path)
    parser.add_argument("elf", type=Path)
    parser.add_argument("--base", type=lambda x: int(x, 0), default=0x80000400)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = compare(args.dump.read_bytes(), args.elf.read_bytes(), args.base)
    report.update(dump=str(args.dump.resolve()), elf=str(args.elf.resolve()), base=hex(args.base))
    output = json.dumps(report, indent=2, ensure_ascii=False)
    if args.output:
        args.output.write_text(output + "\n", encoding="utf-8")
    print(output)
    return 0 if report["match"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

