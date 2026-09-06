#!/usr/bin/env python3
"""generate_robot_params.py — derive slam_robot<N>.yaml and nav2_robot<N>.yaml
for N in 1..4 from the robot_0 originals in experiments/nav/.

WHY THIS EXISTS
---------------
explore.launch.py derives BOTH param filenames from its robot_id argument:

    slam_params_file  default  <EXP_NAV>/slam_robot<id>.yaml
    nav2_params_file  default  <EXP_NAV>/nav2_robot<id>.yaml

Commit 363bb6a supplied slam_robot1..4.yaml, which is enough for a mapper-only
peer (nav2:=false).  It did NOT supply nav2_robot1..4.yaml.  B1.5b brings all
five robots up with nav2:=true, so four of the five launches would die on a
missing params file before any lifecycle node was reached — a failure that
would read, from the console, exactly like the Nav2 activation failure the run
is meant to measure.  This script closes that gap.

THE TRANSFORM
-------------
Both robot_0 files state the rule in their own header:

    slam_robot0.yaml:15  "For robots 1-4 copy this file to slam_robotN.yaml
                          and s/robot_0/robot_N/g"
    nav2_robot0.yaml:21  "For robots 1-4 copy to nav2_robotN.yaml
                          and s/robot_0/robot_N/g"

Comparing slam_robot0.yaml against the shipped slam_robot1.yaml shows the
transform actually applied in August was slightly wider than that sed:

  1. robot_0 -> robot_N        (frames, topics, prose)
  2. robot0  -> robotN         (the filename on line 1 only, since that is the
                               single place the underscore-less form occurs)
  3. the two-line "For robots 1-4 copy ..." note is REPLACED by a two-line
     "GENERATED ... Do NOT hand-edit" note, inserted AFTER the substitutions
     (it survives with 'slam_robot0.yaml' and 's/robot_0/' intact, which is
     only possible if it was written last).

This script implements exactly that, in that order.

THE SELF-TEST
-------------
slam_robot1..4.yaml already exist and have been in use since 6 August.  Run
with --check, the script regenerates them in memory and compares byte for
byte.  If all four come back IDENTICAL, the transform encoded here is the one
that produced the files already known to work, and the nav2 output it writes is
therefore convention, not invention.  If any comes back DIFFERS, this script is
wrong and its nav2 output must not be trusted — inspect the reported diff
before doing anything else.

--check writes nothing.  --write writes only files for robots 1..4; the two
robot_0 sources are never opened for writing.

USAGE
-----
    python3 generate_robot_params.py --check      # verify + preview, no writes
    python3 generate_robot_params.py --write      # write the 8 derived files
    python3 generate_robot_params.py --write --only nav2   # nav2 files only

Pure stdlib.  No ROS, no venv, no sourcing required.
"""

from __future__ import annotations

import argparse
import difflib
import re
import sys
from pathlib import Path

ROBOTS = (1, 2, 3, 4)

# ---------------------------------------------------------------------------
# The note swap, per family.  `old` must appear EXACTLY ONCE in the robot_0
# source; anything else means the source has been edited since this script was
# written and the script must stop rather than guess.
# ---------------------------------------------------------------------------

SLAM_OLD_NOTE = (
    "# For robots 1-4 copy this file to slam_robotN.yaml and s/robot_0/robot_N/g; the\n"
    "# explore.launch.py robot_id arg selects the matching file."
)
SLAM_NEW_NOTE = (
    "# GENERATED from slam_robot0.yaml by s/robot_0/robot_{n}/g. Do NOT hand-edit:\n"
    "# edit slam_robot0.yaml and regenerate all four. explore.launch.py picks the file by robot_id."
)

NAV2_OLD_NOTE = (
    "# For robots 1-4 copy to nav2_robotN.yaml and s/robot_0/robot_N/g."
)
NAV2_NEW_NOTE = (
    "# GENERATED from nav2_robot0.yaml by s/robot_0/robot_{n}/g. Do NOT hand-edit:\n"
    "# edit nav2_robot0.yaml and regenerate all four. explore.launch.py picks the file by robot_id."
)

FAMILIES = {
    "slam": {
        "source": "slam_robot0.yaml",
        "target": "slam_robot{n}.yaml",
        "old_note": SLAM_OLD_NOTE,
        "new_note": SLAM_NEW_NOTE,
        "preexisting": True,   # 363bb6a shipped these; they are the self-test
    },
    "nav2": {
        "source": "nav2_robot0.yaml",
        "target": "nav2_robot{n}.yaml",
        "old_note": NAV2_OLD_NOTE,
        "new_note": NAV2_NEW_NOTE,
        "preexisting": False,  # the gap this script fills
    },
}

# Sentinel placed where the note was, so the note text itself — which contains
# the literal 'robot_0' and 'robot0' — is not caught by the substitutions.
SENTINEL = "\x00__GENERATED_NOTE__\x00"

# robot_0 / robot0, capturing the optional underscore so it is preserved.
ROBOT_RE = re.compile(r"robot(_?)0")


def render(source_text: str, n: int, old_note: str, new_note: str) -> str:
    """Apply the August transform to `source_text` for robot `n`."""
    occurrences = source_text.count(old_note)
    if occurrences != 1:
        raise SystemExit(
            f"ABORT: expected the 'For robots 1-4 ...' note exactly once in the "
            f"source, found {occurrences}. The source header has changed since "
            f"this script was written; update the note constants and re-verify "
            f"with --check before writing anything."
        )

    staged = source_text.replace(old_note, SENTINEL)
    staged = ROBOT_RE.sub(lambda m: f"robot{m.group(1)}{n}", staged)
    result = staged.replace(SENTINEL, new_note.format(n=n))

    # No robot_0 reference may survive outside the generated note, which
    # legitimately names its own source file and sed expression.
    body = result.replace(new_note.format(n=n), "")
    stray = ROBOT_RE.findall(body)
    if stray:
        raise SystemExit(
            f"ABORT: {len(stray)} robot_0/robot0 reference(s) survived "
            f"substitution for robot_{n} outside the generated note. Refusing "
            f"to emit a file that would silently point robot_{n} at robot_0's "
            f"frames and topics."
        )
    return result


def unified(a: str, b: str, name_a: str, name_b: str, limit: int = 40) -> str:
    lines = list(difflib.unified_diff(
        a.splitlines(keepends=True), b.splitlines(keepends=True),
        fromfile=name_a, tofile=name_b, n=1))
    if len(lines) > limit:
        lines = lines[:limit] + [f"... ({len(lines) - limit} more diff lines)\n"]
    return "".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true",
                      help="regenerate in memory and compare; write nothing")
    mode.add_argument("--write", action="store_true",
                      help="write the derived files for robots 1-4")
    ap.add_argument("--dir", type=Path, default=Path(__file__).resolve().parent,
                    help="experiments/nav directory (default: this script's own)")
    ap.add_argument("--only", choices=sorted(FAMILIES), default=None,
                    help="restrict to one family (default: both)")
    args = ap.parse_args()

    nav_dir: Path = args.dir
    if not nav_dir.is_dir():
        raise SystemExit(f"ABORT: not a directory: {nav_dir}")

    families = [args.only] if args.only else sorted(FAMILIES)
    identical = differs = missing = written = 0

    for fam in families:
        spec = FAMILIES[fam]
        src = nav_dir / spec["source"]
        if not src.is_file():
            raise SystemExit(f"ABORT: source not found: {src}")
        source_text = src.read_text()

        print(f"\n=== {fam}: {spec['source']} "
              f"({'self-test — files exist' if spec['preexisting'] else 'GAP — files missing'}) ===")

        for n in ROBOTS:
            target = nav_dir / spec["target"].format(n=n)
            generated = render(source_text, n, spec["old_note"], spec["new_note"])

            if target.is_file():
                existing = target.read_text()
                if existing == generated:
                    identical += 1
                    print(f"  IDENTICAL  {target.name}  ({len(generated)} bytes)")
                    continue
                differs += 1
                print(f"  DIFFERS    {target.name}  "
                      f"(on disk {len(existing)} bytes, generated {len(generated)})")
                print(unified(existing, generated, f"disk/{target.name}",
                              f"generated/{target.name}"))
                if args.write:
                    print(f"  SKIPPED writing {target.name}: it exists and differs. "
                          f"Resolve the diff above by hand.")
                continue

            missing += 1
            print(f"  MISSING    {target.name}  (would write {len(generated)} bytes)")
            if args.write:
                target.write_text(generated)
                written += 1
                print(f"  WROTE      {target.name}")

    print(f"\nsummary: identical={identical} differs={differs} "
          f"missing={missing} written={written}")

    if differs:
        print("\nVERDICT: FAIL — the transform in this script does not reproduce "
              "files already on disk. Do not trust any output it wrote.")
        return 1
    if args.check and identical and not missing:
        print("\nVERDICT: PASS — every existing file reproduces byte for byte.")
    elif args.check:
        print("\nVERDICT: PASS on existing files; listed MISSING files are the gap. "
              "Re-run with --write to create them.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
