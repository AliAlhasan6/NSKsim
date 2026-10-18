#!/usr/bin/env python3
"""The stage that judges the B2 schema: G-S1, then §9's matching and §10's eight
predictions.

docs/specs/SPEC_b2_schema.md v0.2, §9 and §10, registered at `5bf3ee2`. It
follows S13's pattern (divergence_walls.py, "Parked-robot shadows"): before
anything is loaded it refuses an edited tree, requires the registration to be an
ancestor of HEAD, stores the registered text verbatim and finds in that text
every number it applies. Then G-S1's three checks, in order; then, and only
then, §9's matching on the 20 maps and §10's verdicts.

WHERE TRUTH ENTERS (D-S5). Here, and nowhere else in the schema. §2-§7 are
schema_walls.py, which reads no truth and whose per-map files hold none; this
stage writes those files, reads them back, and from then on §9 sees only what
the files hold. The world is read through `graph_walls.sdf_wall_faces`, which
stops if the SDF's face totals move.

WHAT THIS FILE OWNS, AND WHAT IT BORROWS. It owns §9's `truth`, `match_faces`
and `judge_map`, G-S1's three checks and §10's verdicts. Everything else is
borrowed and is not rewritten here:

    schema_walls.schema_of          §2-§7 for one extraction
    schema_walls.schema_for_map     §8's record, and write_record
    divergence_walls.load_side      one map, with §3's two conditions
    divergence_walls.corpus_stems   §10's 20 rows
    divergence_walls._git, _sha256, _is_ancestor, _write_s9
    graph_walls.sdf_wall_faces      §1, the 12 long faces and the SDF's rects
    graph_walls.validate_map        A9, for check 2: A9's own code, not a copy
    graph_walls.face_samples, elements_in_world, read_robot_radius
    graph_walls.extract_with_elements, Params, cell_centre, cell_index

THE CHECK CORPORA ARE RESTATED, NOT IMPORTED. G-S1 check 1 reproduces the counts
of two pinned scratch outputs, so their worlds, their rasterisation and their
lattice phases are restated below (`check_world`, `build_grid`) as S13 restates
M10's synthetic corpus. The scratch files are gitignored; what makes the restated
corpus the pinned one is that the counts match, and the pinned files are read by
hash. Their judging radii (0.5, 0.5 and 0.3 m) are constants of that corpus and
are NOT registered: §9 matches through faces and no element is matched by a
radius (D-S6). They decide only which pinned count a synthetic finding belongs
to, on 160 grids half of which use a small world §9 cannot judge.

IMPORTS. numpy, graph_walls, schema_walls and divergence_walls at module scope;
`fit_world_transform` and `robot_divergence`, which shell out to git for the
spawn table, inside the two functions that need them, as run_s13 does, so a
machine with no bags can still import this.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))

import divergence_walls as dw                                 # noqa: E402
import graph_walls as gw                                      # noqa: E402
import schema_walls as sw                                     # noqa: E402

OUT_DIR_DEFAULT = gw.LOGS_DIR / 'schema'
SPLICE_DIR_DEFAULT = gw.LOGS_DIR / 'splice'
SPEC_REL = 'docs/specs/SPEC_b2_schema.md'
WORLD_SDF_REL = 'ros2_ws/src/nsk_swarm/worlds/knowledge_world.sdf'

# ── the registration (§10, "The stage") ──────────────────────────────────────
SCHEMA_REGISTRATION = '5bf3ee2'    # registered §9, §10, G-S1 and P-S1 to P-S8
SCHEMA_SPEC_NAME = 'SPEC_b2_schema v0.2, §9 (Def S7) and §10 (G-S1, P-S1-P-S8)'

# Ruling 4 (Ali, 17 Oct 2026): §1-§10 at HEAD must be byte-identical to the
# registered text. Ancestry alone does not catch a later edit.
REGISTERED_FROM = '## 1. The world'
REGISTERED_TO = '## 11. Open, before the stage'

GATE_HEADING = '### Gate G-S1'
PREDICTION_LABELS = {
    'P-S1': '**P-S1, inside corners found.**',
    'P-S2': '**P-S2, no wrong inside corner.**',
    'P-S3': '**P-S3, slabs found.**',
    'P-S4': '**P-S4, no wrong slab.**',
    'P-S5': '**P-S5, slab ends found.**',
    'P-S6': '**P-S6, openings at full coverage.**',
    'P-S7': '**P-S7, no unmatched-part inside corner.**',
    'P-S8': '**P-S8, no unmatched-part slab.**',
}
PREDICTIONS = tuple(sorted(PREDICTION_LABELS))
FOUND_PREDICTIONS = {'P-S1': 'inside corners', 'P-S3': 'slabs',
                     'P-S5': 'slab ends'}
MAPS_PREDICTIONS = {'P-S2': ('maps', 'no inside corner is wrong'),
                    'P-S4': ('maps', 'no pair and no slab end is wrong'),
                    'P-S6': ('cut1200 maps', 'there is no opening'),
                    'P-S7': ('maps', 'no inside corner is unmatched-part'),
                    'P-S8': ('maps',
                             'no pair and no slab end is unmatched-part')}

S0_HEADING = '## 0. Decisions recorded here'
S2_HEADING = '## 2. Axis ends (Def S1)'
S4_HEADING = '## 4. Openings (Def S3)'
S5_HEADING = '## 5. Inside corners (Def S4)'
S6_HEADING = '## 6. Slabs: back-to-back pairs (Def S5)'
S7_HEADING = '## 7. Slab ends and outside corners (Def S6)'
S9_HEADING = '## 9. Matching to truth (Def S7, S-O3)'
S10_HEADING = '## 10. Predictions (S-O4)'
M1_HEADING = '### M1, face match'
M4_HEADING = '### M4, slab ends and outside corners'
CHECKS_HEADING = '### On the checks'
PREDS_HEADING = '### The predictions'
TRUTH_LABEL = '- **Truth (§1),**'
DECISIONS = {'D-S5': '**D-S5, truth after the fact**',
             'D-S6': '**D-S6, matched through faces**',
             'D-S7': '**D-S7, rows**'}

# ── the four pinned files ────────────────────────────────────────────────────
# G-S1 check 1 is an equality against two scratch outputs and the report that
# cites them; check 2 is an equality against one A8-A11 run.
A9_FILE = '_validation_20261004T021555Z.json'
A9_SHA256 = ('555ef95f2cc6608c40c925fa1b56e717a80f731f93fc6a06122c841bffc6'
             'babe')
SYNTH2_OUT = 'schema_v0_1_synth2_out.txt'
SYNTH2_SHA256 = ('364def1b37a395dfc1913958a3014915c424e25f7148eadd83af1f3ae9c'
                 'a3056')
CHECK_MD = 'schema_check_draft2.md'
CHECK_MD_SHA256 = ('ed8bdb275866b0a743aa1555cc76960925974853493d7412ca62fcb0f'
                   '740f91b')
MATCH_D2_OUT = 'schema_v0_2_match_d2_out.txt'
MATCH_D2_SHA256 = ('943f7a92e4520985a7864422ce8d0fc5493c5ce9d2806c92391a22a1a'
                   'adf5033')
CHECK_MD_SENTENCE = ('**4.1 Inside corners:** found 64 of 64 in every world and '
                     'variant (640 of 640), none missed.')

# ── constants of the pinned check corpus. NOT REGISTERED. ────────────────────
CHECK_RHO = 0.1               # the grids' rho, as the Margin item states it
CHECK_G = 0.22                # g, the same value the maps carry
CHECK_EPS_M = 1e-6            # "just inside the face", the scratch's EPSI
CHECK_OFFSETS = (0.0, 0.03, 0.05, 0.07)      # 16 lattice phases per world
V1_VARIANTS = ('base', 'thickE', 'thickS', 'jogE', 'jogS')
V1_WORLDS = ('small', 'sdf')
V1_HALF = {'small': 3.5, 'sdf': 10.5}
D2_VARIANTS = V1_VARIANTS + ('blur2', 'endcap', 'jog2')
D2_HALF = 11.5                # room for the world shifted by a spawn
SMALL_WORLD = {'inner': 2.95, 'slabs': [('slab', -1.0, 1.5, 0.9, 1.1)]}
# Ruling 1 (Ali, 17 Oct 2026): the draft-2 check's radii, restated as corpus
# constants. They say which pinned count a synthetic finding belongs to; §9
# matches through faces (D-S6) and never by a radius.
CHECK1_RADII = {'corner': 0.5, 'pair': 0.5, 'outside': 0.3}
# The one quantity check 1 does not compare exactly; see `_close`.
UNROUNDED_TOL_M = 1e-14

# ── what the stage applies, asserted against the found values ───────────────
TAU_M_CELLS = gw.TRUTH_TOL_CELLS     # 2.0: A8's and A9's tolerance
TAU_END_CELLS = TAU_M_CELLS + 0.5    # tau_m + rho/2
STATE_LETTER = {'occupied': 'O', 'unknown': 'U', 'free': 'F'}
BOUNDARY_FACE = {'wall_north': 'N', 'wall_south': 'S', 'wall_east': 'E',
                 'wall_west': 'W'}
OUTWARD_SUFFIX = {'south': '-y', 'north': '+y', 'west': '-x', 'east': '+x'}
ITEM_START = re.compile(r'^(?:[-*] |\d+\. )')


class SchemaGateClosed(RuntimeError):
    """A map's matching asked for before G-S1 allowed it."""


def die(msg: str) -> None:
    """Refuse, at exit 2. The callers' convention in this directory."""
    print(f'schema_judge: {msg}', file=sys.stderr)
    sys.exit(2)


# ─────────────────────── reading the registered text ─────────────────────────

def spec_section(text: str, heading: str) -> str:
    """From an exact heading line to the next heading at the same or a higher level.

    `spec_paragraph` cannot read this spec: the predictions, G-S1's checks and
    the tolerance bullets are list items with no blank line between them, so a
    paragraph read from one would run into the next. Sections are sliced first,
    then one item at a time.
    """
    level = len(heading) - len(heading.lstrip('#'))
    lines = text.splitlines()
    start = None
    for i, ln in enumerate(lines):
        if ln.rstrip() == heading:
            start = i
            break
    if start is None:
        die(f'the spec carries no heading {heading!r}')
    out = [lines[start]]
    for ln in lines[start + 1:]:
        rest = ln.lstrip('#')
        depth = len(ln) - len(rest)
        if depth and depth <= level and rest.startswith(' '):
            break
        out.append(ln)
    return '\n'.join(out) + '\n'


def spec_item(text: str, label: str) -> str:
    """One list item, from its bold label through its continuation lines.

    It stops at a blank line, at the next `- ` or `N. ` marker, or at a heading,
    and joins the lines with one space as `spec_paragraph` does. Every regex in
    `schema_constants_from` runs over one item, so a number cannot be read from
    a neighbour.
    """
    lines = text.splitlines()
    start = None
    for i, ln in enumerate(lines):
        s = ln.strip()
        if s.startswith(label) or (ITEM_START.match(s)
                                   and ITEM_START.sub('', s).startswith(label)):
            start = i
            break
    if start is None:
        die(f'the spec carries no item starting {label!r}')
    out = [ITEM_START.sub('', lines[start].strip())]
    for ln in lines[start + 1:]:
        s = ln.strip()
        if not s or ITEM_START.match(s) or s.startswith('#'):
            break
        out.append(s)
    return ' '.join(out)


def spec_block(text: str, start: str, end: str) -> str:
    """A block kept as the spec has it, line breaks and all."""
    i = text.find(start)
    j = text.find(end, i + 1) if i >= 0 else -1
    if i < 0 or j < 0:
        die(f'the spec has no block from {start!r} to {end!r}')
    return text[i:j].strip('\n')


def registered_body(text: str) -> str:
    """§1 to §10, the text ruling 4 compares between HEAD and the registration."""
    i = text.find(REGISTERED_FROM)
    j = text.find(REGISTERED_TO, i + 1) if i >= 0 else -1
    if i < 0 or j < 0:
        die(f'the spec has no §1-§10 body ({REGISTERED_FROM!r} to '
            f'{REGISTERED_TO!r})')
    return text[i:j]


def schema_verbatim(text: str) -> dict:
    """The registered text, read from the spec by script and never retyped."""
    s0 = spec_section(text, S0_HEADING)
    s2 = spec_section(text, S2_HEADING)
    s4 = spec_section(text, S4_HEADING)
    s5 = spec_section(text, S5_HEADING)
    s6 = spec_section(text, S6_HEADING)
    s7 = spec_section(text, S7_HEADING)
    s9 = spec_section(text, S9_HEADING)
    s10 = spec_section(text, S10_HEADING)
    m1 = spec_section(s9, M1_HEADING)
    m4 = spec_section(s9, M4_HEADING)
    checks = spec_section(s9, CHECKS_HEADING)
    gate = spec_section(s10, GATE_HEADING)
    preds = spec_section(s10, PREDS_HEADING)
    items = {
        'rho': spec_item(m1, '**Margin.**'),
        'eps_tau_c': spec_item(s5, '**Tolerance.**'),
        'g': spec_item(s4, '**Opening.**'),
        's_max': spec_item(s6, '**Tolerance.**'),
        'tau_e': spec_item(s7, '**Aligned.**'),
        'tau_m': spec_item(m1, '**Tolerance.**'),
        'tau_end': spec_item(m4, '**Tolerance.**'),
        'axis_tie': spec_item(s2, '**Axis.**'),
        'm0': spec_item(s9, '**M0, frame.**'),
        'slab_ends_in_this_world': spec_item(s7, '**In this world**'),
        'gate_1_synthetic': spec_item(gate, '**The synthetic grids.**'),
        'gate_2_frame_and_truth': spec_item(gate,
                                            '**Frame and truth source.**'),
        'gate_3_decoy': spec_item(gate, '**X-S1, a decoy truth.**'),
        'decoy_on_the_checks': spec_item(
            checks, '**The same grids against a decoy truth,**'),
        'rows': spec_item(s10, '**Rows.**'),
        'verdicts': spec_item(s10, '**Verdicts in integers.**'),
        'stage': spec_item(s10, '**The stage**'),
    }
    for name, label in PREDICTION_LABELS.items():
        items[name] = spec_item(preds, label)
    for name, label in DECISIONS.items():
        items[name] = spec_item(s0, label)
    return {'blocks': {'9': s9, '10': s10,
                       'truth': spec_block(s9, TRUTH_LABEL, M1_HEADING),
                       'world_heading': next(
                           ln for ln in text.splitlines()
                           if ln.startswith('## 1. The world'))},
            'items': items}


def _one(pattern: str, item: str, what: str) -> tuple:
    m = re.search(pattern, item)
    if not m:
        die(f'the registered item for {what} is not the shape the stage reads: '
            f'{item!r}')
    return m.groups()


def _manifest_names(manifest: str) -> tuple[set, dict]:
    """Scan "The stage"'s bracketed list, one item at a time.

    Every item must be one of four shapes. A number added to the code without a
    finder is caught by the set comparison in `schema_constants_from`; a number
    in this list the stage cannot read is caught here.
    """
    shapes = (('name', r'`([a-z_]+)`'),
              ('decoy', r"the decoy's shift"),
              ('tie', r'1e-9'),
              ('bound', r'(\d+) of (\d+) for (P-S\d(?:(?:, | and )P-S\d)*)'))
    sep = re.compile(r'(?:,\s+|\s+and\s+)')
    names, bounds, pos = set(), {}, 0
    while pos < len(manifest):
        for kind, pattern in shapes:
            m = re.compile(pattern).match(manifest, pos)
            if not m:
                continue
            if kind == 'name':
                names.add(m.group(1))
            elif kind == 'decoy':
                names.add('decoy shift')
            elif kind == 'tie':
                names.add('1e-9')
            else:
                for p in re.findall(r'P-S\d', m.group(3)):
                    names.add(p)
                    bounds[p] = (int(m.group(1)), int(m.group(2)))
            pos = m.end()
            break
        else:
            die(f'"The stage" names a number the stage cannot read, at '
                f'{manifest[pos:pos + 40]!r}')
        m = sep.match(manifest, pos)
        if m:
            pos = m.end()
        elif pos < len(manifest):
            die(f'"The stage"\'s list does not separate its items at '
                f'{manifest[pos:pos + 40]!r}')
    return names, bounds


def schema_constants_from(verbatim: dict) -> dict:
    """Every number the stage applies, found in the registered text or refused."""
    it = {k: ' '.join(v.split()) for k, v in verbatim['items'].items()}
    found: dict = {}

    (rho,) = _one(r"the maps' `rho` = ([0-9.]+)", it['rho'], 'rho')
    (grid_rho,) = _one(r'`rho` = ([0-9.]+) as the grids use it', it['rho'],
                       "the check corpus's rho")
    (largest, margin) = _one(
        r'the largest `m\(S\)` distance was ([0-9.]+) m .*'
        r'`tau_m` minus it is \*\*([0-9.]+) m\*\*', it['rho'], 'the M1 margin')
    found['rho'] = {'maps': float(rho), 'grids': float(grid_rho),
                    'largest_m_perp_m': float(largest),
                    'tau_m_margin_m': float(margin)}

    (tau_c, eps_half) = (_one(r'`tau_c = eps \+ rho` \(([0-9.]+) m\)',
                              it['eps_tau_c'], 'tau_c')[0],
                         _one(r'`eps \+ rho/2` \(([0-9.]+) m\)',
                              it['eps_tau_c'], 'eps')[0])
    found['tau_c'] = {'value_m': float(tau_c), 'formula': 'eps + rho'}
    found['eps'] = {'eps_plus_half_rho_m': float(eps_half),
                    'formula': 'graph_walls.Params.eps(rho)'}
    (g,) = _one(r'greater than `g` \(([0-9.]+) m\)', it['g'], 'g')
    found['g'] = {'value_m': float(g)}
    (s_max,) = _one(r'`s_max = ([0-9.]+) m`', it['s_max'], 's_max')
    found['s_max'] = {'value_m': float(s_max)}
    (tau_e,) = _one(r'`tau_e` \(`tau_e = rho`, ([0-9.]+) m\)', it['tau_e'],
                    'tau_e')
    found['tau_e'] = {'value_m': float(tau_e), 'formula': 'rho'}
    (tau_m,) = _one(r'`tau_m = 2\*rho` \(([0-9.]+) m\), '
                    r'`graph_walls.TRUTH_TOL_CELLS`', it['tau_m'], 'tau_m')
    found['tau_m'] = {'value_m': float(tau_m), 'formula': '2*rho',
                      'cells': TAU_M_CELLS}
    (tau_end,) = _one(r'`tau_end = tau_m \+ rho/2` \(([0-9.]+) m\)',
                      it['tau_end'], 'tau_end')
    found['tau_end'] = {'value_m': float(tau_end),
                        'formula': 'tau_m + rho/2', 'cells': TAU_END_CELLS}

    # 1e-9 twice: check 2's tolerance and §2's axis tie. The manifest names it
    # once, so the stage holds both under the one name.
    (tie,) = _one(r'differ by at most ([0-9.e+-]+), S has no axis',
                  it['axis_tie'], "§2's axis tie")
    gate_tol, n_a9, of_a9 = _one(r'within ([0-9.e+-]+) on (\d+) of (\d+) maps',
                                 it['gate_2_frame_and_truth'], "check 2")
    found['1e-9'] = {'axis_tie': float(tie), 'gate': float(gate_tol),
                     'gate_maps': (int(n_a9), int(of_a9))}

    shift = _one(r'shifted by \(\+([0-9.]+), \+([0-9.]+)\) m',
                 it['gate_3_decoy'], "the decoy's shift")
    shift_again = _one(r'shifted by \(\+([0-9.]+), \+([0-9.]+)\) m',
                       it['decoy_on_the_checks'], "the decoy's shift (§9)")
    n_decoy, of_decoy = _one(r'on (\d+) of (\d+) maps', it['gate_3_decoy'],
                             "check 3")
    if shift != shift_again:
        die(f'G-S1 item 3 shifts the world by {shift} and §9 by {shift_again}')
    found['decoy shift'] = {'m': (float(shift[0]), float(shift[1])),
                            'maps': (int(n_decoy), int(of_decoy))}

    # The predictions: each bound in both forms, the item and the Verdicts
    # paragraph, and the two must agree.
    verd = {
        'found': {'P-S1': _one(r'P-S1 and P-S3 hold when found ≥ (\d+)',
                               it['verdicts'], "P-S1's verdict")[0],
                  'P-S5': _one(r'P-S5 when found ≥ (\d+)', it['verdicts'],
                               "P-S5's verdict")[0]},
        'maps': {'wrong': _one(r'P-S2 and P-S4 when at most (\d+) map has a '
                               r'wrong element', it['verdicts'],
                               "P-S2's verdict")[0],
                 'opening': _one(r'P-S6 when at most (\d+) cut1200 map has an '
                                 r'opening', it['verdicts'],
                                 "P-S6's verdict")[0],
                 'unmatched': _one(r'P-S7 and P-S8 when at most (\d+) map has '
                                   r'an unmatched-part element', it['verdicts'],
                                   "P-S7's verdict")[0]}}
    verd['found']['P-S3'] = verd['found']['P-S1']
    at_most = {'P-S2': verd['maps']['wrong'], 'P-S4': verd['maps']['wrong'],
               'P-S6': verd['maps']['opening'],
               'P-S7': verd['maps']['unmatched'],
               'P-S8': verd['maps']['unmatched']}
    preds: dict = {}
    for name in PREDICTIONS:
        item = it[name]
        for other in PREDICTIONS:
            if other != name and other in item:
                die(f"{name}'s item names {other}: {item!r}")
        if name in FOUND_PREDICTIONS:
            lo, of, kind = _one(
                r'On the five cut1200 maps, at least (\d+) of the (\d+) true '
                r'(.+) are found\.', item, name)
            if kind != FOUND_PREDICTIONS[name]:
                die(f'{name} registers "{kind}", and the stage counts '
                    f'{FOUND_PREDICTIONS[name]}')
            if int(verd['found'][name]) != int(lo):
                die(f'{name} reads "at least {lo}" and the Verdicts paragraph '
                    f'"found ≥ {verd["found"][name]}"')
            preds[name.lower()] = {'rule': 'found', 'kind': kind,
                                   'min': int(lo), 'of': int(of),
                                   'rows': 'cut1200'}
        else:
            rows, what = MAPS_PREDICTIONS[name]
            lo, of, tail = _one(r'On at least (\d+) of the (\d+) ' + rows
                                + r', (.+)\.', item, name)
            if tail != what:
                die(f'{name} registers "{tail}", and the stage judges "{what}"')
            if int(of) - int(lo) != int(at_most[name]):
                die(f'{name} reads "{lo} of the {of}" and the Verdicts '
                    f'paragraph "at most {at_most[name]}"')
            preds[name.lower()] = {'rule': 'maps', 'what': what,
                                   'min': int(lo), 'of': int(of),
                                   'at_most': int(of) - int(lo),
                                   'rows': 'cut1200'
                                           if rows.startswith('cut1200')
                                           else 'all'}
    # Keyed in lower case, as S13 keys P18 and P19: the fail path bans the
    # registered names as keys, and these are found before a map is read.
    found['predictions'] = preds

    # The manifest: exactly the numbers the stage has finders for.
    (manifest,) = _one(r'every number it applies \((.*?)\) and refuses on a '
                       r'mismatch', it['stage'], "the stage's manifest")
    named, bounds = _manifest_names(manifest)
    mine = {k for k in found if k != 'predictions'} | {p.upper()
                                                       for p in preds}
    if named != mine:
        die(f'"The stage" names {sorted(named)} and the stage finds '
            f'{sorted(mine)}')
    for name, (lo, of) in bounds.items():
        got = preds[name.lower()]
        if (lo, of) != (got['min'], got['of']):
            die(f'the manifest gives {name} {lo} of {of} and its item '
                f'{got["min"]} of {got["of"]}')

    # Everything else the stage reads from the registered text.
    rows_n, pattern, k0, k1, cuts = _one(
        r"The (\d+) maps of the divergence spec's §2, `([^`]+)`, K from (\d+) "
        r'to (\d+) and C in \{([0-9, ]+)\}', it['rows'], "§10's rows")
    found['rows'] = {'n': int(rows_n), 'pattern': pattern,
                     'k': (int(k0), int(k1)),
                     'cuts': tuple(int(c) for c in cuts.split(','))}
    (spawn_rev,) = _one(r'`DOT_POSES` at `([0-9a-f]+)`', it['m0'], "M0's frame")
    found['spawn_rev'] = spawn_rev
    truth_block = ' '.join(verbatim['blocks']['truth'].split())
    n_faces, n_in, n_slabs, n_out = (
        _one(r'the (\d+) long observable world faces', truth_block, 'the faces')[0],
        _one(r'the (\d+) inside corners', truth_block, 'the inside corners')[0],
        _one(r'the (\d+) slabs', truth_block, 'the slabs')[0],
        _one(r'the (\d+) outside corners', truth_block, 'the outside corners')[0])
    n_ends, n_out_7 = _one(r'the true slab ends are (\d+), and the true outside '
                           r'corners (\d+)', it['slab_ends_in_this_world'],
                           "§7's totals")
    if int(n_out) != int(n_out_7):
        die(f'§9 names {n_out} outside corners and §7 {n_out_7}')
    found['truth_counts'] = {'faces': int(n_faces), 'inside_corners': int(n_in),
                             'slabs': int(n_slabs), 'slab_ends': int(n_ends),
                             'outside_corners': int(n_out)}
    (gap_m,) = _one(r'Same-facing long faces are at least ([0-9.]+) m apart',
                    truth_block, 'the same-facing gap')
    found['truth_counts']['min_same_facing_gap_m'] = float(gap_m)
    (sdf_sha,) = _one(r'`knowledge_world\.sdf`, sha256 `([0-9a-f]+)…`',
                      verbatim['blocks']['world_heading'], "the world's sha256")
    (a9_file,) = _one(r'([A-Za-z0-9_]*_validation_[0-9A-Z]+\.json)',
                      it['tau_m'], "A8's stored file")
    (a9_sha,) = _one(r'sha256 `([0-9a-f]+)…`', it['tau_m'], "A8's file sha256")
    md, n_160, found_640, of_640 = _one(
        r'`(schema_check_draft2\.md)` on its (\d+) grids \((\d+) of (\d+) '
        r'inside corners', it['gate_1_synthetic'], "check 1's 160 grids")
    d2_out, n_640 = _one(r'`(schema_v0_2_match_d2_out\.txt)` on its (\d+) '
                         r'grids', it['gate_1_synthetic'],
                         "check 1's 640 grids")
    found['check1'] = {'md': md, 'n_160': int(n_160), 'd2_out': d2_out,
                       'n_640': int(n_640),
                       'corners_640': (int(found_640), int(of_640))}
    found['files'] = {'sdf_sha256_prefix': sdf_sha, 'a9_file': a9_file,
                      'a9_sha256_prefix': a9_sha}

    # The constants the code holds equal the found values, as S13 checks its
    # own. The ones that depend on a map's rho -- tau_c, tau_e, tau_m, tau_end
    # and eps -- are checked against every map in `map_thresholds`.
    for what, got, want in (
            ('schema_walls.AXIS_TIE', sw.AXIS_TIE, found['1e-9']['axis_tie']),
            ('schema_walls.S_MAX_M', sw.S_MAX_M, found['s_max']['value_m']),
            ('graph_walls.TRUTH_TOL_CELLS', gw.TRUTH_TOL_CELLS,
             found['tau_m']['cells']),
            ("the stage's A9 file", A9_FILE, found['files']['a9_file']),
            ("check 1's report", CHECK_MD, found['check1']['md']),
            ("check 1's 640-grid output", MATCH_D2_OUT,
             found['check1']['d2_out'])):
        if got != want:
            die(f'{what} is {got!r} and the registered text gives {want!r}')
    if not A9_SHA256.startswith(found['files']['a9_sha256_prefix']):
        die(f'the stage pins the A9 file at {A9_SHA256}, and §9 M1 names '
            f'{found["files"]["a9_sha256_prefix"]}…')
    assert TAU_END_CELLS == TAU_M_CELLS + 0.5 == 2.5
    return found


# ───────────────────────────── §9: truth (Def S7) ────────────────────────────

def axis_of_normal(n) -> int:
    """The `graph_walls.DIRS` index of an axis-aligned outward normal."""
    dots = sw.AX @ np.asarray(n, dtype=float)
    return int(np.argmax(dots))


def truth(world: dict, shift=(0.0, 0.0)) -> dict:
    """§9's truth: 12 long faces, 4 inside corners, 4 slabs, 16 outside corners.

    From `graph_walls.sdf_wall_faces`, which stops if the SDF's face totals
    change: `inward` gives the 12 long observable faces, `interior` clips a
    boundary face from the SDF's 20.2 m to the exposed 19.9 m between the two
    inside corners on it, and `rects` gives the slabs and their end caps.

    `shift` moves the whole world, and is (0, 0) for the real truth and
    (+0.5, +0.5) for X-S1's decoy.
    """
    sx, sy = float(shift[0]), float(shift[1])
    xw, xe, ys, yn = world['interior']
    faces = {}
    for f in world['inward']:
        ax = 'y' if f['n'][0] == 0.0 else 'x'
        v = f['p0'][1] if ax == 'y' else f['p0'][0]
        if f['model'] in BOUNDARY_FACE:
            name = BOUNDARY_FACE[f['model']]
            lo, hi = (xw, xe) if ax == 'y' else (ys, yn)
        else:
            name = f['model'] + OUTWARD_SUFFIX[f['side']]
            i = 0 if ax == 'y' else 1
            lo, hi = sorted((f['p0'][i], f['p1'][i]))
        along = sx if ax == 'y' else sy
        faces[name] = {'name': name, 'source': f'{f["model"]}.{f["side"]}',
                       'ax': ax, 'axis': axis_of_normal(f['n']),
                       'n': [float(f['n'][0]), float(f['n'][1])],
                       'v': v + (sy if ax == 'y' else sx),
                       'lo': lo + along, 'hi': hi + along,
                       'length_m': float(hi - lo)}
    corners = []
    for sgx, x in ((-1, xw), (1, xe)):
        for sgy, y in ((-1, ys), (1, yn)):
            corners.append({'name': ('E' if sgx > 0 else 'W')
                                    + ('N' if sgy > 0 else 'S'),
                            'p': [x + sx, y + sy],
                            'faces': ('E' if sgx > 0 else 'W',
                                      'N' if sgy > 0 else 'S')})
    slabs = []
    for name, (x0, x1, y0, y1) in world['rects']:
        if not name.startswith('maze_'):
            continue
        horiz = (x1 - x0) > (y1 - y0)
        if horiz:
            faces_of = (name + '-y', name + '+y')
            ends = (x0 + sx, x1 + sx)
            outside = ([[x0 + sx, y0 + sy], [x0 + sx, y1 + sy]],
                       [[x1 + sx, y0 + sy], [x1 + sx, y1 + sy]])
        else:
            faces_of = (name + '-x', name + '+x')
            ends = (y0 + sy, y1 + sy)
            outside = ([[x0 + sx, y0 + sy], [x1 + sx, y0 + sy]],
                       [[x0 + sx, y1 + sy], [x1 + sx, y1 + sy]])
        slabs.append({'name': name, 'horiz': horiz, 'faces': faces_of,
                      'ends': ends, 'outside': outside})
    gaps = []
    names = sorted(faces)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            fa, fb = faces[a], faces[b]
            if fa['axis'] != fb['axis']:
                continue
            if min(fa['hi'], fb['hi']) - max(fa['lo'], fb['lo']) > 0:
                gaps.append((abs(fa['v'] - fb['v']), a, b))
    counts = {'faces': len(faces), 'inside_corners': len(corners),
              'slabs': len(slabs), 'slab_ends': 2 * len(slabs),
              'outside_corners': 4 * len(slabs)}
    return {'shift': [sx, sy], 'faces': faces, 'corners': corners,
            'slabs': slabs, 'counts': counts,
            'min_same_facing_gap': min(gaps) if gaps else None}


def check_spawn_rev(constants: dict) -> str:
    """M0's `DOT_POSES` revision against the one the corpus was built at.

    `robot_divergence.SPAWN_REV` is 08617b2, the b2maps era; bag_overlap's own
    default is the b16 ring and would put every map 3.15 m away.
    """
    import robot_divergence as rd                             # noqa: PLC0415
    if rd.SPAWN_REV != constants['spawn_rev']:
        die(f'M0 names DOT_POSES at {constants["spawn_rev"]} and '
            f'robot_divergence uses {rd.SPAWN_REV}')
    return rd.SPAWN_REV


def check_truth(truth_: dict, constants: dict) -> None:
    """The truth the stage built is the truth §9 and §7 describe."""
    want = dict(constants['truth_counts'])
    gap = want.pop('min_same_facing_gap_m')
    if truth_['counts'] != want:
        die(f'the world gives {truth_["counts"]}, and §9 registers {want}')
    if truth_['min_same_facing_gap'][0] < gap:
        die(f'same-facing faces lie {truth_["min_same_facing_gap"]} apart, '
            f'under §9\'s {gap} m')


# ──────────────────────────── §9: M1 and M2-M6 ───────────────────────────────

def _perp(p, f) -> float:
    return (p[1] if f['ax'] == 'y' else p[0]) - f['v']


def _along(p, f) -> float:
    return p[0] if f['ax'] == 'y' else p[1]


def _world(p, shift) -> list:
    return [p[0] + shift[0], p[1] + shift[1]]


def match_faces(record: dict, truth_: dict, shift, tau_m: float) -> dict:
    """M1, for one per-map record: segment id -> world face name, or None.

    W's outward normal is a(S); `m(S)` lies within `tau_m` of W's line; S's
    interval and W's exposed extent overlap, both projected on W's direction.
    More than one qualifying face goes to the nearest line (then, as the pinned
    check broke the tie, to the name), which in this world cannot arise.
    """
    out = {}
    for s in record['segments']:
        ends = s['axis_ends']
        if ends is None:
            continue
        m = _world(ends['m'], shift)
        q0, q1 = _world(ends['q0'], shift), _world(ends['q1'], shift)
        best = None
        for f in truth_['faces'].values():
            if f['axis'] != s['axis']:
                continue
            dm = _perp(m, f)
            if abs(dm) > tau_m:
                continue
            a0, a1 = sorted((_along(q0, f), _along(q1, f)))
            if min(a1, f['hi']) - max(a0, f['lo']) <= 0:
                continue
            cand = (abs(dm), f['name'])
            if best is None or cand < best:
                best = cand
        out[s['id']] = None if best is None else best[1]
    return out


def _spurious(faces: list) -> str:
    """§9: unmatched-part if a segment matched no face, else wrong."""
    return 'unmatched_part' if any(f is None for f in faces) else 'wrong'


def _empty_kind() -> dict:
    return {'found': {}, 'missed': [], 'wrong': [], 'unmatched_part': [],
            'duplicates': []}


def _end_key(slab: str, which: int) -> str:
    """One of a slab's two ends, named so the results file stays JSON."""
    return f'{slab}:{which}'


def judge_map(record: dict, truth_: dict, shift, tol: dict) -> dict:
    """M1 to M6 for one per-map record. Reads the record and truth, nothing else.

    M2 and M3: a finding is true for an element when its two segments match that
    element's two faces, one each; otherwise it is spurious, of one of the two
    kinds. M4 assigns each end of a true pair to the nearer end of its slab and
    calls it true within `tau_end`; a slab end of a true pair beyond `tau_end`
    is wrong. M5: every opening is spurious. M6 reports why a true element was
    missed, and never judges it.
    """
    fm = match_faces(record, truth_, shift, tol['tau_m'])
    faces, seg = truth_['faces'], {s['id']: s for s in record['segments']}
    rep = {'m_perp': [], 'axis_end_perp': [], 'c_to_corner': [],
           'c_to_corner_cheb': [], 'slab_end_along': [], 'outside_corner': [],
           'true_pair_ends': {}}
    for sid, fname in fm.items():
        if fname is None:
            continue
        f, ends = faces[fname], seg[sid]['axis_ends']
        rep['m_perp'].append(abs(_perp(_world(ends['m'], shift), f)))
        rep['axis_end_perp'].append(max(
            abs(_perp(_world(ends['q0'], shift), f)),
            abs(_perp(_world(ends['q1'], shift), f))))

    corners = _empty_kind()
    for i, k in enumerate(record['inside_corners']):
        a, b = k['segments']
        fa, fb = fm.get(a), fm.get(b)
        hit = [t for t in truth_['corners'] if {fa, fb} == set(t['faces'])]
        rec = {'corner': i, 'segments': [a, b], 'faces': [fa, fb]}
        if not hit:
            corners[_spurious([fa, fb])].append(rec)
            continue
        t = hit[0]
        c = _world(k['c'], shift)
        d = [c[0] - t['p'][0], c[1] - t['p'][1]]
        rec.update({'true': t['name'], 'states': k['states'],
                    'c_to_corner_m': math.hypot(*d),
                    'c_to_corner_cheb_m': max(abs(d[0]), abs(d[1]))})
        rep['c_to_corner'].append(rec['c_to_corner_m'])
        rep['c_to_corner_cheb'].append(rec['c_to_corner_cheb_m'])
        if t['name'] in corners['found']:
            corners['duplicates'].append(rec)
        else:
            corners['found'][t['name']] = rec

    slabs, ends = _empty_kind(), _empty_kind()
    for i, p in enumerate(record['pairs']):
        a, b = p['segments']
        fa, fb = fm.get(a), fm.get(b)
        hit = [t for t in truth_['slabs'] if {fa, fb} == set(t['faces'])]
        rec = {'pair': i, 'segments': [a, b], 'faces': [fa, fb],
               'separation_m': p['separation_m']}
        kind = 'true' if hit else _spurious([fa, fb])
        if not hit:
            slabs[kind].append(rec)
        elif hit[0]['name'] in slabs['found']:
            slabs['duplicates'].append(dict(rec, true=hit[0]['name']))
        else:
            slabs['found'][hit[0]['name']] = dict(rec, true=hit[0]['name'])
        for j, e in enumerate(p['ends']):
            if hit:
                key = str((e['aligned'], e['state']))
                rep['true_pair_ends'][key] = \
                    rep['true_pair_ends'].get(key, 0) + 1
            if not e['slab_end']:
                continue                     # §7 records it; it is no finding
            erec = {'pair': i, 'end': j, 'segments': [a, b],
                    'faces': [fa, fb], 'record': e['record']}
            if not hit:
                ends[kind].append(erec)
                continue
            t = hit[0]
            ax = 0 if t['horiz'] else 1
            mean = (e['points'][0][ax] + e['points'][1][ax]) / 2.0 + shift[ax]
            which = 0 if (abs(mean - t['ends'][0])
                          <= abs(mean - t['ends'][1])) else 1
            err = abs(mean - t['ends'][which])
            erec.update({'slab': t['name'], 'slab_end': which,
                         'along_error_m': err})
            if err > tol['tau_end']:
                erec['reason'] = 'the pair is true and the end is beyond tau_end'
                ends['wrong'].append(erec)
                continue
            rep['slab_end_along'].append(err)
            oc = [_world(q, shift) for q in e['points']]
            erec['outside_corners'] = [
                {'p': q, 'to_box_corner_m': min(math.dist(q, z)
                                                for z in t['outside'][which])}
                for q in oc]
            rep['outside_corner'] += [z['to_box_corner_m']
                                      for z in erec['outside_corners']]
            if _end_key(t['name'], which) in ends['found']:
                ends['duplicates'].append(erec)
            else:
                ends['found'][_end_key(t['name'], which)] = erec

    openings = [{'gap': i, 'segments': g['segments'], 'width_m': g['width_m'],
                 'state': g['state'],
                 'faces': [fm.get(j) for j in g['segments']]}
                for i, g in enumerate(record['gaps']) if g['opening']]

    # ── M6: missed, and why ────────────────────────────────────────────────
    matched = {f for f in fm.values() if f is not None}
    for t in truth_['corners']:
        if t['name'] in corners['found']:
            continue
        miss = {'corner': t['name'], 'faces': list(t['faces']),
                'n_faces_matched': sum(f in matched for f in t['faces'])}
        if miss['n_faces_matched'] == 2:
            miss['nearest_axis_end_m'] = {
                f: min((min(math.dist(_world(seg[s]['axis_ends'][e], shift),
                                      t['p']) for e in ('q0', 'q1'))
                        for s, g in fm.items() if g == f), default=None)
                for f in t['faces']}
        corners['missed'].append(miss)
    for t in truth_['slabs']:
        if t['name'] not in slabs['found']:
            slabs['missed'].append(
                {'slab': t['name'], 'faces': list(t['faces']),
                 'n_faces_matched': sum(f in matched for f in t['faces'])})
        true_pair = slabs['found'].get(t['name'])
        for which in (0, 1):
            if _end_key(t['name'], which) in ends['found']:
                continue
            miss = {'slab': t['name'], 'slab_end': which,
                    'slab_has_a_true_pair': true_pair is not None,
                    'section_7_record': None}
            if true_pair is not None:
                p = record['pairs'][true_pair['pair']]
                ax = 0 if t['horiz'] else 1
                for e in p['ends']:
                    mean = ((e['points'][0][ax] + e['points'][1][ax]) / 2.0
                            + shift[ax])
                    near = 0 if (abs(mean - t['ends'][0])
                                 <= abs(mean - t['ends'][1])) else 1
                    if near == which:
                        miss['section_7_record'] = e['record']
                        miss['along_error_m'] = abs(mean - t['ends'][which])
            ends['missed'].append(miss)

    entry = {s['id']: s['line'] for s in record['segments']}
    per_face = {}
    for name in sorted(faces):
        mine = [s for s, f in fm.items() if f == name]
        per_face[name] = {'n_segments': len(mine),
                          'n_lines_entries': len({entry[s] for s in mine})}
    return {'faces': fm, 'unmatched_segments': sum(f is None
                                                   for f in fm.values()),
            'inside_corners': corners, 'slabs': slabs, 'slab_ends': ends,
            'openings': openings,
            'reported': dict(rep, per_face=per_face,
                             split_faces=sorted(
                                 n for n, v in per_face.items()
                                 if v['n_lines_entries'] > 1),
                             multi_segment_faces={
                                 n: v['n_segments'] for n, v in
                                 per_face.items() if v['n_segments'] > 1})}


def map_counts(j: dict) -> dict:
    """The per-map counts §10 judges, from one `judge_map`."""
    def of(kind):
        return {'found': len(j[kind]['found']), 'missed': len(j[kind]['missed']),
                'wrong': len(j[kind]['wrong']),
                'unmatched_part': len(j[kind]['unmatched_part']),
                'duplicates': len(j[kind]['duplicates'])}
    return {'inside_corners': of('inside_corners'), 'slabs': of('slabs'),
            'slab_ends': of('slab_ends'), 'openings': len(j['openings']),
            'unmatched_segments': j['unmatched_segments']}


# ──────────────────── the pinned check corpus, restated ──────────────────────

def check_world(kind: str, world: dict | None = None) -> dict:
    """One world of the pinned checks: 'sdf' from the SDF, 'small' restated.

    The faces are the 4 boundary inner faces over their exposed extent and all
    16 slab faces, each as (name, axis, line, inward sign, lo, hi). Half of the
    160 grids use the small world, which §9 cannot judge; its geometry is a
    constant of the pinned corpus and is not registered.
    """
    if kind == 'small':
        inner, slabs = SMALL_WORLD['inner'], list(SMALL_WORLD['slabs'])
    else:
        xw, xe, ys, yn = world['interior']
        if not (xe == -xw == yn == -ys):
            die(f'the arena interior {world["interior"]} is not the square the '
                'pinned check corpus was built on')
        inner = xe
        slabs = [(n, *r) for n, r in world['rects'] if n.startswith('maze_')]
    faces = [('N', 'y', inner, +1, -inner, inner),
             ('S', 'y', -inner, -1, -inner, inner),
             ('E', 'x', inner, +1, -inner, inner),
             ('W', 'x', -inner, -1, -inner, inner)]
    for n, x0, x1, y0, y1 in slabs:
        faces += [(n + '-y', 'y', y0, +1, x0, x1),
                  (n + '+y', 'y', y1, -1, x0, x1),
                  (n + '-x', 'x', x0, +1, y0, y1),
                  (n + '+x', 'x', x1, -1, y0, y1)]
    return {'kind': kind, 'inner': inner, 'slabs': slabs, 'faces': faces,
            'true_in': [(sx * inner, sy * inner)
                        for sx in (-1, 1) for sy in (-1, 1)],
            'true_out': [(x, y) for _n, x0, x1, y0, y1 in slabs
                         for x in (x0, x1) for y in (y0, y1)],
            'blur_slab': slabs[0]}


def _ci(v: float, origin: float, rho: float) -> int:
    return int(gw.cell_index(np.array([float(v)]), origin, rho)[0])


def build_grid(world: dict, shift, half: float, ox: float, oy: float,
               variant: str, rho: float = CHECK_RHO):
    """One synthetic grid: the world less `shift`, at lattice phase (ox, oy).

    Rasterisation, as the pinned check states it: a cell is OCC if it holds a
    point just inside an observable face (where a beam ends), FREE if it
    overlaps free space and is not OCC, UNKNOWN otherwise. The variants blur a
    face one or two cells on its free side (thickE, blur2), half a face (jogE,
    jog2, jogS), the first slab's -y face (thickS) or its two end caps (endcap).
    """
    sx, sy = float(shift[0]), float(shift[1])
    org = (-half + ox + rho / 2.0, -half + oy + rho / 2.0)
    n = int(round(2 * half / rho))
    occ = np.zeros((n, n), dtype=bool)
    cells = {}
    for name, ax, v, sg, lo, hi in world['faces']:
        if ax == 'y':
            v, lo, hi = v - sy, lo - sx, hi - sx
            r = _ci(v + sg * CHECK_EPS_M, org[1], rho)
            cs = list(range(_ci(lo + CHECK_EPS_M, org[0], rho),
                            _ci(hi - CHECK_EPS_M, org[0], rho) + 1))
            occ[r, cs] = True
            cells[name] = (r, cs)
        else:
            v, lo, hi = v - sx, lo - sy, hi - sy
            c = _ci(v + sg * CHECK_EPS_M, org[0], rho)
            rs = list(range(_ci(lo + CHECK_EPS_M, org[1], rho),
                            _ci(hi - CHECK_EPS_M, org[1], rho) + 1))
            occ[rs, c] = True
            cells[name] = (c, rs)
    if variant in ('thickE', 'jogE', 'blur2', 'jog2'):
        c, rs = cells['E']
        ys = gw.cell_centre(org[1], np.array(rs), rho)
        keep = [r for r, y in zip(rs, ys)
                if variant in ('thickE', 'blur2') or y > -sy]
        occ[keep, c - 1] = True
        if variant in ('blur2', 'jog2'):
            occ[keep, c - 2] = True
    if variant in ('thickS', 'jogS'):
        bn, x0s, x1s = world['blur_slab'][:3]
        r, cs = cells[bn + '-y']
        xs = gw.cell_centre(org[0], np.array(cs), rho)
        mid = (x0s + x1s) / 2.0 - sx
        keep = [c for c, x in zip(cs, xs) if variant == 'thickS' or x > mid]
        occ[r - 1, keep] = True
    if variant == 'endcap':
        bn = world['blur_slab'][0]
        for side, dc in (('-x', -1), ('+x', +1)):
            c, rs = cells[bn + side]
            occ[rs, c + dc] = True
    inner = world['inner']
    i = np.arange(n)
    x0 = org[0] + (i - 0.5) * rho
    y0 = org[1] + (i - 0.5) * rho

    def ov(a0, a1, b0, b1):
        return np.clip(np.minimum(a1, b1) - np.maximum(a0, b0), 0, None)

    free = np.outer(ov(y0, y0 + rho, -inner - sy, inner - sy),
                    ov(x0, x0 + rho, -inner - sx, inner - sx))
    for _n, bx0, bx1, by0, by1 in world['slabs']:
        free -= np.outer(ov(y0, y0 + rho, by0 - sy, by1 - sy),
                         ov(x0, x0 + rho, bx0 - sx, bx1 - sx))
    grid = np.full((n, n), gw.UNKNOWN, dtype=np.uint8)
    grid[free > 1e-12] = gw.FREE
    grid[occ] = gw.OCC
    return grid, org


def grid_schema(world: dict, shift, half: float, ox: float, oy: float,
                variant: str, params, rho: float = CHECK_RHO) -> dict:
    """§2-§7 on one synthetic grid, through graph_walls' own extraction."""
    grid, org = build_grid(world, shift, half, ox, oy, variant, rho)
    graph, el, seg_of = gw.extract_with_elements(grid, rho, org, params)
    return sw.schema_of(graph, el, seg_of, grid, rho, org, params)


def _nearest(p, pts) -> tuple[int, float]:
    d = [math.dist(p, q) for q in pts]
    i = min(range(len(d)), key=lambda j: d[j])
    return i, d[i]


def _to_rect(p, rect) -> float:
    x0, x1, y0, y1 = rect
    return math.hypot(max(x0 - p[0], 0.0, p[0] - x1),
                      max(y0 - p[1], 0.0, p[1] - y1))


def _fmt(vals) -> str:
    """The pinned scripts' `mx`: the largest of a list to four decimals."""
    return f'{max(vals):.4f}' if vals else 'n/a'


def _range(v) -> tuple[str, str]:
    return (f'{min(v):.4f}', f'{max(v):.4f}') if v else ('n/a', 'n/a')


# ────────────────────── G-S1 check 1: the pinned counts ──────────────────────

def judge_160(world: dict, variant: str, params, radii: dict = CHECK1_RADII,
              offsets=CHECK_OFFSETS, rho: float = CHECK_RHO) -> dict:
    """§2-§7 on one (world, variant) block of the 160 grids, judged by radius.

    The radii are the pinned check's, not the spec's (ruling 1): a found corner
    belongs to the true inside corner within `corner`, a pair to the slab whose
    rectangle both midpoints lie within `pair`, and an outside corner to the
    true one within `outside`.
    """
    T = {'grids': 0, 'true_in': 0, 'no_axis': 0, 'found': 0, 'missed': 0,
         'false_corners': 0, 'states': {}, 'near': [], 'far': [],
         'c_true': [], 'q_true': [], 'pairs': {}, 'slab_missed': {},
         'false_pairs': 0, 'ends': [], 'end_states': {}, 'outside': [],
         'false_ends': 0, 'gaps': 0, 'openings': 0}
    for ox in offsets:
        for oy in offsets:
            rec = grid_schema(world, (0.0, 0.0), world_half(world), ox, oy,
                              variant, params, rho)
            seg = {s['id']: s for s in rec['segments']}
            T['grids'] += 1
            T['true_in'] += len(world['true_in'])
            T['no_axis'] += (rec['counts']['segments']
                             - rec['counts']['with_axis_ends'])
            T['gaps'] += rec['counts']['gaps']
            T['openings'] += rec['counts']['openings']
            hit = set()
            for k in rec['inside_corners']:
                i, d = _nearest(k['c'], world['true_in'])
                if d > radii['corner']:
                    T['false_corners'] += 1
                    continue
                hit.add(i)
                T['found'] += 1
                key = ''.join(STATE_LETTER[x] for x in k['states'])
                T['states'][key] = T['states'].get(key, 0) + 1
                T['near'].append(min(k['near_m']))
                T['far'].append(max(k['near_m']))
                T['c_true'].append(d)
                for sid, end in zip(k['segments'], k['near_end']):
                    T['q_true'].append(math.dist(seg[sid]['axis_ends'][end],
                                                 world['true_in'][i]))
            T['missed'] += len(world['true_in']) - len(hit)
            mine = set()
            for p in rec['pairs']:
                ma = seg[p['segments'][0]]['axis_ends']['m']
                mb = seg[p['segments'][1]]['axis_ends']['m']
                own = [z for z in world['slabs']
                       if _to_rect(ma, z[1:]) <= radii['pair']
                       and _to_rect(mb, z[1:]) <= radii['pair']]
                if not own:
                    T['false_pairs'] += 1
                    continue
                mine.add(own[0][0])
                T['pairs'].setdefault(own[0][0], []).append(p['separation_m'])
                for e in p['ends']:
                    T['ends'].append(e['diff_m'])
                    key = str((e['aligned'], e['state']))
                    T['end_states'][key] = T['end_states'].get(key, 0) + 1
                    for cpt in e['outside_corners']:
                        _i, d = _nearest(cpt, world['true_out'])
                        T['outside'].append(d)
                        if d > radii['outside']:
                            T['false_ends'] += 1
            for z in world['slabs']:
                if z[0] not in mine:
                    T['slab_missed'].setdefault(z[0], []).append((ox, oy))
    return T


def world_half(world: dict) -> float:
    """The half-width the pinned 160-grid check built each world at."""
    return V1_HALF[world['kind']]


def as_160_block(T: dict) -> dict:
    """One `judge_160` as the fields the pinned text states, formatted as it is."""
    near, far = _range(T['near']), _range(T['far'])
    q = _range(T['q_true'])
    out_r = _range(T['outside'])
    return {
        'no_axis': T['no_axis'],
        'corners': (T['found'], T['true_in'], T['missed']),
        'false_corners': T['false_corners'],
        'distances': (near[0], near[1], far[0], far[1], _fmt(T['c_true']),
                      q[0], q[1]),
        'states': T['states'],
        'slabs': {n: (len(v), T['grids'], f'{min(v):.4f}', f'{max(v):.4f}')
                  for n, v in sorted(T['pairs'].items())},
        'slab_missed': sorted(T['slab_missed']),
        'false_pairs': T['false_pairs'],
        'ends': (len(T['ends']), _fmt(T['ends']), T['end_states'],
                 len(T['outside']), out_r[0], out_r[1], T['false_ends']),
        'gaps': (T['gaps'], T['openings']),
    }


def parse_160_out(text: str) -> dict:
    """The pinned 160-grid output, block by block. An unread line is a refusal."""
    head = re.compile(r'=== (\w+), (\w+): (\d+) grids, tau_c ([0-9.]+), '
                      r's_max ([0-9.]+) ===$')
    pats = {
        'no_axis': r'segments without axis ends: (\d+)$',
        'corners': r'inside corners found (\d+) of (\d+), missed (\d+) \[.*\]$',
        'distances': r'nearer axis end -> c ([0-9.]+)\.\.([0-9.]+); farther '
                     r'axis end -> c ([0-9.]+)\.\.([0-9.]+); c -> true '
                     r'([0-9.]+) max; axis end -> true corner ([0-9.]+)\.\.'
                     r'([0-9.]+)$',
        'states': r'axis end states at found corners (\{.*\})$',
        'false_corners': r'false inside corners (\[.*\]); other junctions '
                         r'(\[.*\])$',
        'slab': r'slab (\S+): pairs found (\d+) of (\d+), separation '
                r'([0-9.]+)\.\.([0-9.]+)$',
        'slab_missed': r'slab (\S+): MISSED in (\d+) grids .*$',
        'false_pairs': r'false pairs (\[.*\])$',
        'ends': r'slab ends examined (\d+): alignment max ([0-9.]+); '
                r'\(aligned, state\) (\{.*\}); outside corners (\d+), '
                r'distance to true outside corner ([0-9.]+)\.\.([0-9.]+); '
                r'false (\[.*\])$',
        'gaps': r'gaps formed \(segments consecutive on one entry\) (\d+); '
                r'openings (\d+)$',
        'tilt': r'tilt of segments with risers .*$',
        'own_diff': r'own-cell rule differences near true inside corners '
                    r'.*$',
    }
    pats = {k: re.compile(v) for k, v in pats.items()}
    blocks, cur = {}, None
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        m = head.match(line)
        if m:
            cur = {'tau_c': float(m.group(4)), 's_max': float(m.group(5)),
                   'grids': int(m.group(3)), 'slabs': {}, 'slab_missed': []}
            blocks[(m.group(1), m.group(2))] = cur
            continue
        if cur is None:
            die(f'{SYNTH2_OUT}: a line before the first block: {line!r}')
        for key, pat in pats.items():
            m = pat.match(line)
            if not m:
                continue
            if key == 'slab':
                cur['slabs'][m.group(1)] = (int(m.group(2)), int(m.group(3)),
                                            m.group(4), m.group(5))
            elif key == 'slab_missed':
                cur['slab_missed'].append(m.group(1))
            elif key == 'false_corners':
                cur['false_corners'] = 0 if m.group(1) == '[]' else -1
            elif key == 'false_pairs':
                cur['false_pairs'] = 0 if m.group(1) == '[]' else -1
            elif key == 'states':
                cur['states'] = ast.literal_eval(m.group(1))
            elif key == 'ends':
                cur['ends'] = (int(m.group(1)), m.group(2),
                               {str(k): v for k, v in
                                ast.literal_eval(m.group(3)).items()},
                               int(m.group(4)), m.group(5), m.group(6),
                               0 if m.group(7) == '[]' else -1)
            elif key == 'no_axis':
                cur['no_axis'] = int(m.group(1))
            elif key == 'corners':
                cur['corners'] = tuple(int(g) for g in m.groups())
            elif key == 'distances':
                cur['distances'] = m.groups()
            elif key == 'gaps':
                cur['gaps'] = (int(m.group(1)), int(m.group(2)))
            break
        else:
            die(f'{SYNTH2_OUT}: this line is not one the stage reads: {line!r}')
    need = ('no_axis', 'corners', 'distances', 'states', 'false_corners',
            'slabs', 'false_pairs', 'ends', 'gaps')
    short = {k: [f for f in need if f not in v] for k, v in blocks.items()}
    short = {k: v for k, v in short.items() if v}
    if len(blocks) != len(V1_WORLDS) * len(V1_VARIANTS) or short:
        die(f'{SYNTH2_OUT}: {len(blocks)} blocks, '
            f'{len(V1_WORLDS) * len(V1_VARIANTS)} expected; incomplete '
            f'{short}')
    return blocks


def parse_640_out(text: str) -> dict:
    """The pinned 640-grid output: header, eight blocks and the footer."""
    lines = [ln.rstrip() for ln in text.splitlines() if ln.strip()]
    head = {}
    head['tolerances'] = _one(r'^tau_m ([0-9.]+)  tau_end ([0-9.]+)  '
                              r'spawns (\[.*\])$', lines[0], 'the d2 header')
    head['truth'] = _one(r'^truth: (\d+) long faces, (\d+) inside corners, '
                         r'(\d+) slabs, (\d+) slab ends$', lines[1],
                         "the d2 header's truth")
    head['gap'] = _one(r'^closest same-facing, overlapping long faces: '
                       r'(\(.*\))$', lines[2], "the d2 header's gap")
    pats = {
        'segments': r'segments without axis ends (\d+); segments matching no '
                    r'face (\d+)$',
        'corners': r'inside corners found/missed/wrong/unmatched-part '
                   r'(\[.*\]) of (\d+)$',
        'slabs': r'slabs +found/missed/wrong/unmatched-part (\[.*\]) of '
                 r'(\d+)$',
        'ends': r'slab ends +found/missed/wrong/unmatched-part (\[.*\]) of '
                r'(\d+); ends of true pairs \(aligned, state\) (\{.*\})$',
        'duplicates': r'duplicates \(corners, slabs, slab ends\) (\[.*\])$',
        'openings': r'openings (\d+); faces on >1 lines\[\] entry (.+); faces '
                    r'with >1 segment (.+)$',
        'max': r'max: m\(S\) perp (\S+) \| axis end perp (\S+) \| c->corner '
               r'(\S+) \(cheb (\S+)\) \| slab end along (\S+) \| outside '
               r'corner (\S+)$',
        'control': r'control \(\+([0-9.]+), \+([0-9.]+)\): segments matched '
                   r'(\d+); corners/slabs/ends found (\[.*\]); '
                   r'wrong/unmatched-part corners (\[.*\])$',
        'largest': r'largest m\(S\) distance, unrounded: (\S+)$',
    }
    pats = {k: re.compile(v) for k, v in pats.items()}
    foot = {'all': re.compile(r'largest m\(S\) distance over all (\d+) grids, '
                              r'unrounded: (\S+) \((.+)\)$'),
            'margin': re.compile(r'tau_m - largest, rho = (\S+?)'
                                 r"(?: \(the maps' YAML\))?: ([0-9.]+) m "
                                 r'\((\S+)\)$')}
    blocks, cur, tail = {}, None, {}
    for line in lines[3:]:
        m = re.match(r'=== (\w+): (\d+) grids \(5 spawns x 16 phases\) ===$',
                     line)
        if m:
            cur = {'grids': int(m.group(2))}
            blocks[m.group(1)] = cur
            continue
        m = foot['all'].match(line)
        if m:
            tail['largest'] = (int(m.group(1)), m.group(2), m.group(3))
            cur = None
            continue
        m = foot['margin'].match(line)
        if m:
            tail.setdefault('margins', []).append(m.groups())
            continue
        if cur is None:
            die(f'{MATCH_D2_OUT}: a line outside every block: {line!r}')
        for key, pat in pats.items():
            m = pat.match(line)
            if not m:
                continue
            if key in ('corners', 'slabs'):
                cur[key] = (ast.literal_eval(m.group(1)), int(m.group(2)))
            elif key == 'ends':
                cur[key] = (ast.literal_eval(m.group(1)), int(m.group(2)),
                            {str(k): v for k, v in
                             ast.literal_eval(m.group(3)).items()})
            elif key == 'duplicates':
                cur[key] = ast.literal_eval(m.group(1))
            elif key == 'openings':
                cur[key] = (int(m.group(1)),
                            {} if m.group(2) == 'none'
                            else ast.literal_eval(m.group(2)),
                            {} if m.group(3) == 'none'
                            else ast.literal_eval(m.group(3)))
            elif key == 'segments':
                cur[key] = (int(m.group(1)), int(m.group(2)))
            elif key == 'control':
                cur[key] = ((float(m.group(1)), float(m.group(2))),
                            int(m.group(3)), ast.literal_eval(m.group(4)),
                            ast.literal_eval(m.group(5)))
            else:
                cur[key] = m.groups()
            break
        else:
            die(f'{MATCH_D2_OUT}: this line is not one the stage reads: '
                f'{line!r}')
    if sorted(blocks) != sorted(D2_VARIANTS) or 'largest' not in tail \
            or len(tail.get('margins', [])) != 2:
        die(f'{MATCH_D2_OUT}: blocks {sorted(blocks)} and footer '
            f'{sorted(tail)} are not the pinned shape')
    return {'head': head, 'blocks': blocks, 'tail': tail}


def judge_640(world: dict, truth_now: dict, truth_decoy: dict, variant: str,
              spawn: list, params, tol: dict, offsets=CHECK_OFFSETS,
              rho: float = CHECK_RHO) -> dict:
    """§2-§7 and §9 on one variant's 80 grids, against truth and the decoy."""
    A = {'grids': 0, 'no_axis': 0, 'unmatched': 0, 'corners': [0, 0, 0, 0],
         'slabs': [0, 0, 0, 0], 'ends': [0, 0, 0, 0], 'end_states': {},
         'duplicates': [0, 0, 0], 'openings': 0, 'split': {},
         'multi_segment': {}, 'm_perp': [], 'axis_end_perp': [],
         'c_to_corner': [], 'c_to_corner_cheb': [], 'slab_end_along': [],
         'outside_corner': [], 'control_matched': 0,
         'control_found': [0, 0, 0], 'control_corners': [0, 0]}
    for s in spawn:
        for ox in offsets:
            for oy in offsets:
                rec = grid_schema(world, s, D2_HALF, ox, oy, variant, params,
                                  rho)
                A['grids'] += 1
                A['no_axis'] += (rec['counts']['segments']
                                 - rec['counts']['with_axis_ends'])
                j = judge_map(rec, truth_now, s, tol)
                c = map_counts(j)
                A['unmatched'] += j['unmatched_segments']
                for key, name in (('corners', 'inside_corners'),
                                  ('slabs', 'slabs'), ('ends', 'slab_ends')):
                    A[key] = [a + c[name][f] for a, f in
                              zip(A[key], ('found', 'missed', 'wrong',
                                           'unmatched_part'))]
                A['duplicates'] = [
                    a + c[n]['duplicates'] for a, n in
                    zip(A['duplicates'], ('inside_corners', 'slabs',
                                          'slab_ends'))]
                A['openings'] += c['openings']
                rep = j['reported']
                for key, n in rep['true_pair_ends'].items():
                    A['end_states'][key] = A['end_states'].get(key, 0) + n
                for f in rep['split_faces']:
                    A['split'][f] = A['split'].get(f, 0) + 1
                for f in rep['multi_segment_faces']:
                    A['multi_segment'][f] = A['multi_segment'].get(f, 0) + 1
                for key in ('m_perp', 'axis_end_perp', 'c_to_corner',
                            'c_to_corner_cheb', 'slab_end_along',
                            'outside_corner'):
                    A[key] += rep[key]
                d = judge_map(rec, truth_decoy, s, tol)
                dc = map_counts(d)
                A['control_matched'] += sum(f is not None
                                            for f in d['faces'].values())
                A['control_found'] = [
                    a + dc[n]['found'] for a, n in
                    zip(A['control_found'], ('inside_corners', 'slabs',
                                             'slab_ends'))]
                A['control_corners'] = [
                    A['control_corners'][0] + dc['inside_corners']['wrong'],
                    A['control_corners'][1]
                    + dc['inside_corners']['unmatched_part']]
    return A


def as_640_block(A: dict, shift) -> dict:
    """One `judge_640` as the fields the pinned text states, formatted as it is."""
    return {
        'grids': A['grids'],
        'segments': (A['no_axis'], A['unmatched']),
        'corners': (A['corners'], 4 * A['grids']),
        'slabs': (A['slabs'], 4 * A['grids']),
        'ends': (A['ends'], 8 * A['grids'], A['end_states']),
        'duplicates': A['duplicates'],
        'openings': (A['openings'], A['split'], A['multi_segment']),
        'max': (_fmt(A['m_perp']), _fmt(A['axis_end_perp']),
                _fmt(A['c_to_corner']), _fmt(A['c_to_corner_cheb']),
                _fmt(A['slab_end_along']), _fmt(A['outside_corner'])),
        'control': ((float(shift[0]), float(shift[1])), A['control_matched'],
                    A['control_found'], A['control_corners']),
        'largest': max(A['m_perp'], default=None),
    }


def _same(got, want, what: str, out: list) -> bool:
    if got != want:
        out.append({'what': what, 'pinned': want, 'stage': got})
        return False
    return True


def _close(got, want: str, what: str, out: list,
           tol: float = UNROUNDED_TOL_M) -> bool:
    """A float the pinned text printed with `repr`, to within `tol`.

    The one quantity that is not compared exactly. The pinned script added the
    spawn to both axis ends and averaged them; §9 reads `m(S)` from the per-map
    record, where §6 and §8 put it, and carries that by M0. The two agree in
    exact arithmetic and differ in the last bits -- measured at 1.8e-15 m on
    blur2 and at 0 on base -- because the coordinates they add reach 12 m, whose
    own ulp is 1.8e-15. `tol` is 1e-14 m, which still pins thirteen significant
    digits of a 0.09 m distance; every four-decimal maximum above, and every
    count, is compared exactly.
    """
    if got is None or abs(float(got) - float(want)) > tol:
        out.append({'what': what, 'pinned': want, 'stage': repr(got)})
        return False
    return True


def check1_160(path: Path, md_path: Path, params, constants: dict,
               world: dict) -> dict:
    """G-S1 check 1a: §2-§7 reproduce the 160-grid counts of the draft-2 check."""
    text = path.read_text()
    md = md_path.read_text()
    blocks = parse_160_out(text)
    diff: list = []
    if CHECK_MD_SENTENCE not in ' '.join(md.split()):
        die(f'{CHECK_MD} does not say "{CHECK_MD_SENTENCE}"')
    row = re.search(r'\| *' + re.escape(SYNTH2_OUT) + r' *\| `([0-9a-f]{64})` *\|',
                    md)
    if not row:
        die(f'{CHECK_MD} does not pin {SYNTH2_OUT} by sha256')
    if row.group(1) != SYNTH2_SHA256:
        die(f'{CHECK_MD} pins {SYNTH2_OUT} at {row.group(1)}, and the stage '
            f'reads {SYNTH2_SHA256}')
    n_grids = sum(b['grids'] for b in blocks.values())
    if n_grids != constants['check1']['n_160']:
        die(f'{SYNTH2_OUT} holds {n_grids} grids and G-S1 names '
            f'{constants["check1"]["n_160"]}')
    total_found = total_of = 0
    for kind in V1_WORLDS:
        w = check_world(kind, world)
        for variant in V1_VARIANTS:
            want = blocks[(kind, variant)]
            got = as_160_block(judge_160(w, variant, params))
            if want['grids'] != len(CHECK_OFFSETS) ** 2:
                die(f'{SYNTH2_OUT}: {kind}/{variant} is {want["grids"]} grids')
            total_found += got['corners'][0]
            total_of += got['corners'][1]
            before = len(diff)
            for key in ('no_axis', 'corners', 'false_corners', 'distances',
                        'states', 'slabs', 'slab_missed', 'false_pairs',
                        'ends', 'gaps'):
                _same(got[key], want[key], f'{kind}/{variant}.{key}', diff)
            print(f'  check 1a {kind:<5} {variant:<7} corners '
                  f'{got["corners"][0]}/{got["corners"][1]}, pairs '
                  f'{sum(v[0] for v in got["slabs"].values())}, ends examined '
                  f'{got["ends"][0]}, openings {got["gaps"][1]}'
                  + ('' if len(diff) == before
                     else f'   {len(diff) - before} DIFFER'), flush=True)
    want_found, want_of = constants['check1']['corners_640']
    if (total_found, total_of) != (want_found, want_of):
        diff.append({'what': 'the 160 grids\' inside corners',
                     'pinned': [want_found, want_of],
                     'stage': [total_found, total_of]})
    return {'ok': not diff, 'file': path.name, 'sha256': SYNTH2_SHA256,
            'md': md_path.name, 'md_sha256': CHECK_MD_SHA256,
            'n_grids': n_grids, 'n_blocks': len(blocks),
            'inside_corners': [total_found, total_of], 'differing': diff}


def check1_640(path: Path, params, constants: dict, world: dict,
               spawn: list, tol: dict) -> dict:
    """G-S1 check 1b: §2-§7 and §9 reproduce the 640-grid counts of d2."""
    pinned = parse_640_out(path.read_text())
    diff: list = []
    w = check_world('sdf', world)
    shift = constants['decoy shift']['m']
    now, decoy = truth(world), truth(world, shift)
    head = pinned['head']
    _same(head['tolerances'][:2], (f'{tol["tau_m"]:.4f}', f'{tol["tau_end"]:.4f}'),
          'head.tolerances', diff)
    _same(ast.literal_eval(head['tolerances'][2]),
          [tuple(round(float(v), 10) for v in s) for s in spawn],
          'head.spawns', diff)
    _same(tuple(int(v) for v in head['truth']),
          (now['counts']['faces'], now['counts']['inside_corners'],
           now['counts']['slabs'], now['counts']['slab_ends']),
          'head.truth', diff)
    _same(head['gap'][0], repr(now['min_same_facing_gap']), 'head.gap', diff)
    if sum(b['grids'] for b in pinned['blocks'].values()) \
            != constants['check1']['n_640']:
        die(f'{MATCH_D2_OUT} does not hold G-S1\'s '
            f'{constants["check1"]["n_640"]} grids')
    largest, who = None, []
    for variant in D2_VARIANTS:
        want = pinned['blocks'][variant]
        before = len(diff)
        got = as_640_block(judge_640(w, now, decoy, variant, spawn, params,
                                     tol), shift)
        for key in ('segments', 'corners', 'slabs', 'ends', 'duplicates',
                    'openings', 'max', 'control'):
            _same(got[key], want[key], f'{variant}.{key}', diff)
        _close(got['largest'], want['largest'][0], f'{variant}.largest', diff)
        if got['largest'] is not None:
            if largest is None or got['largest'] > largest:
                largest, who = got['largest'], [variant]
            elif got['largest'] == largest:
                who.append(variant)
        print(f'  check 1b {variant:<7} corners {got["corners"][0]}, slabs '
              f'{got["slabs"][0]}, ends {got["ends"][0]}, unmatched '
              f'{got["segments"][1]}, control matched {got["control"][1]}'
              + ('' if len(diff) == before
                 else f'   {len(diff) - before} DIFFER'), flush=True)
    tail = pinned['tail']
    _same((tail['largest'][0], tail['largest'][2]),
          (constants['check1']['n_640'], ', '.join(who)), 'tail.largest', diff)
    _close(largest, tail['largest'][1], 'tail.largest.value', diff)
    _same(round(constants['rho']['largest_m_perp_m'], 10),
          round(float(tail['largest'][1]), 10), 'tail.largest.spec', diff)
    for rho_s, printed, exact in tail['margins']:
        rho_v = float(rho_s)
        _same(f'{2 * rho_v - largest:.7f}', printed,
              f'tail.margin[{rho_s}]', diff)
        _close(2 * rho_v - largest, exact, f'tail.margin[{rho_s}].exact', diff)
        if f'{2 * rho_v - largest:.7f}' != \
                f'{constants["rho"]["tau_m_margin_m"]:.7f}':
            diff.append({'what': f'tail.margin[{rho_s}] against the spec',
                         'pinned': printed,
                         'stage': constants['rho']['tau_m_margin_m']})
    return {'ok': not diff, 'file': path.name, 'sha256': MATCH_D2_SHA256,
            'n_grids': sum(b['grids'] for b in pinned['blocks'].values()),
            'n_blocks': len(pinned['blocks']),
            'largest_m_perp_m': largest, 'largest_at': who, 'differing': diff}


def check1(splice_dir: Path, params, constants: dict, world: dict,
           spawn: list, tol: dict) -> dict:
    """G-S1 check 1, both pinned corpora. 800 extractions; the slow step."""
    a = check1_160(splice_dir / SYNTH2_OUT, splice_dir / CHECK_MD, params,
                   constants, world)
    b = check1_640(splice_dir / MATCH_D2_OUT, params, constants, world, spawn,
                   tol) if a['ok'] else None
    return {'ok': bool(a['ok'] and b is not None and b['ok']),
            'the_160_grids': a, 'the_640_grids': b}


# ───────────────── G-S1 checks 2 and 3, and the door ─────────────────────────

def stored_step(stored: dict) -> float:
    """A9's sample step, read from the stored run rather than fixed here."""
    step = {s['A9']['sample_step_m'] for s in stored.values()}
    if len(step) != 1:
        die(f'the stored A9 run uses {sorted(step)} as its sample step; '
            'check 2 recomputes A9 at the step it was computed with')
    return step.pop()


def a9_of_map(side: dict, spawn: list, world: dict, samples: dict,
              radius: float) -> dict:
    """A8-A11 for one map through `graph_walls.validate_map`, not a copy of it."""
    k, _cut = gw._k_and_cut(side['stem'], 'check 2 needs the robot and the cut')
    elements = gw.elements_in_world(side['el'], side['seg_of'], spawn[k])
    return gw.validate_map(side['graph'], elements, spawn, k, world, samples,
                           side['rho'], radius)


def check2_frame_and_truth(stems: list, stored: dict, a9_of, tol: float,
                           step: float) -> dict:
    """G-S1 check 2: A9's recall recomputed, equal to the stored value.

    Gating: `A9.recall` within `tol` on every map. Beside, reported as `g11_g0`
    reports its own: `recall_of_reachable`, `covered_m` and A8's two shares.
    """
    covers = set(stored) == set(stems) and len(stored) == len(stems)
    maps = []
    for stem in stems:
        got, st = a9_of(stem), stored.get(stem, {})
        a9, a8 = got['A9'], got['A8']
        s9, s8 = st.get('A9', {}), st.get('A8', {})
        maps.append({
            'map': stem,
            'recall': {'stored': s9.get('recall'), 'computed': a9['recall']},
            'equal': (s9.get('recall') is not None
                      and abs(a9['recall'] - s9['recall']) <= tol),
            'sample_step_m': {'stored': s9.get('sample_step_m'),
                              'used': step},
            'beside': {f: {'stored': s9.get(f), 'computed': a9[f]}
                       for f in ('recall_of_reachable', 'covered_m')}
            | {f: {'stored': s8.get(f), 'computed': a8.get(f)}
               for f in ('share_assigned', 'share_all')}})
    n_eq = sum(m['equal'] for m in maps)
    return {'ok': bool(covers and n_eq == len(stems) and len(stems) == 20),
             'covers_the_20_maps': covers, 'n': len(maps), 'n_equal': n_eq,
             'tol': tol, 'sample_step_m': step, 'maps': maps,
             'differing': [m for m in maps if not m['equal']]}


def check3_decoy(records: dict, spawn: list, decoy: dict, tol: dict) -> dict:
    """G-S1 check 3, X-S1: against the shifted world, no segment matches."""
    maps = []
    for stem in sorted(records):
        k, _cut = gw._k_and_cut(stem, 'X-S1 needs the robot and the cut')
        fm = match_faces(records[stem], decoy, spawn[k], tol['tau_m'])
        n = sum(f is not None for f in fm.values())
        maps.append({'map': stem, 'n_with_axis_ends': len(fm),
                     'n_matched': n, 'ok': n == 0})
    return {'ok': bool(len(maps) == 20 and all(m['ok'] for m in maps)),
            'shift_m': decoy['shift'], 'n_maps': len(maps),
            'n_maps_clean': sum(m['ok'] for m in maps), 'maps': maps,
            'differing': [m for m in maps if not m['ok']]}


class SchemaDoor:
    """The one door through which the stage matches a real map to truth.

    `judge(stem)` raises `SchemaGateClosed` until `open_after` has been handed
    G-S1's three checks and all three hold. Check 3 does not come through here:
    it calls `match_faces` against the DECOY directly, which is the gate and not
    a finding, and so does check 1 on its synthetic grids.
    """

    def __init__(self, records: dict, spawn: list, truth_: dict, tol: dict):
        self._records, self._spawn, self._truth = records, spawn, truth_
        self._tol = tol
        self._open = False
        self.asked: list = []

    @property
    def is_open(self) -> bool:
        return self._open

    def open_after(self, *checks) -> bool:
        self._open = (len(checks) == 3 and all(
            c is not None and c.get('ok') is True for c in checks))
        return self._open

    def judge(self, stem: str) -> dict:
        if not self._open:
            raise SchemaGateClosed(f'{stem}: G-S1 has not passed; no map is '
                                   'matched to truth')
        self.asked.append(stem)
        k, _cut = gw._k_and_cut(stem, 'the matching needs the robot')
        return judge_map(self._records[stem], self._truth, self._spawn[k],
                         self._tol)


# ────────────────────────── §10: the verdicts ────────────────────────────────

def found_verdict(n_found: int, bound: dict, rows: list) -> dict:
    """P-S1, P-S3 or P-S5: found >= min over the five cut1200 maps."""
    return {'holds': n_found >= bound['min'], 'n_found': n_found,
            'of': bound['of'], 'min_required': bound['min'],
            'rule': f'found >= {bound["min"]} of {bound["of"]}',
            'rows': rows, 'kind': bound['kind']}


def maps_verdict(maps_with: list, bound: dict, of_rows: int) -> dict:
    """P-S2, P-S4, P-S6, P-S7 or P-S8: at most `of - min` maps have one."""
    if of_rows != bound['of']:
        die(f'{bound["what"]!r} is judged on {of_rows} rows and registered on '
            f'{bound["of"]}')
    return {'holds': len(maps_with) <= bound['at_most'],
            'n_maps_with': len(maps_with), 'of': bound['of'],
            'at_most': bound['at_most'],
            'rule': f'at most {bound["at_most"]} of {bound["of"]} maps',
            'what': bound['what'], 'maps': maps_with}


def verdicts_from(judgments: list, constants: dict, truth_: dict) -> dict:
    """P-S1 to P-S8 in integers, on the rows D-S7 gives each."""
    pred = constants['predictions']
    full = [j for j in judgments if j['cut'] == dw.FULL_CUT]
    per = truth_['counts']
    want = {'P-S1': len(full) * per['inside_corners'],
            'P-S3': len(full) * per['slabs'],
            'P-S5': len(full) * per['slab_ends']}
    out = {}
    for name, kind in (('P-S1', 'inside_corners'), ('P-S3', 'slabs'),
                       ('P-S5', 'slab_ends')):
        bound = pred[name.lower()]
        if bound['of'] != want[name]:
            die(f'{name} is registered over {bound["of"]} true elements and '
                f'the {len(full)} cut1200 maps hold {want[name]}')
        out[name] = found_verdict(
            sum(j[kind]['found'] for j in full), bound,
            [{'map': j['map'], 'found': j[kind]['found'],
              'missed': j[kind]['missed']} for j in full])
    out['P-S2'] = maps_verdict(
        [j['map'] for j in judgments if j['inside_corners']['wrong']],
        pred['p-s2'], len(judgments))
    out['P-S4'] = maps_verdict(
        [j['map'] for j in judgments
         if j['slabs']['wrong'] or j['slab_ends']['wrong']],
        pred['p-s4'], len(judgments))
    out['P-S6'] = maps_verdict([j['map'] for j in full if j['openings']],
                               pred['p-s6'], len(full))
    out['P-S7'] = maps_verdict(
        [j['map'] for j in judgments if j['inside_corners']['unmatched_part']],
        pred['p-s7'], len(judgments))
    out['P-S8'] = maps_verdict(
        [j['map'] for j in judgments
         if j['slabs']['unmatched_part'] or j['slab_ends']['unmatched_part']],
        pred['p-s8'], len(judgments))
    return out


# ───────────────────── the fail path, the meta and the writer ────────────────

# `at_most` and `min` are registered bounds and are part of every meta's
# constants; what a fail file may not hold is a verdict or a map's matching.
BANNED_IN_A_FAIL = {'judgments', 'verdicts', 'reported', 'holds', 'n_holding',
                    'min_required', 'n_maps_with', 'n_found', *PREDICTIONS}


def _no_verdict_in_schema(payload: dict) -> None:
    """Assert a G-S1 fail payload carries no match on a real map and no verdict.

    Exact key names, walked from the root. `spec_verbatim` is the registered
    wording, keyed P-S1 to P-S8 among the rest, and is not walked: it is part of
    every real meta, which is what the S9-S11 fail paths forgot. What a fail
    file may hold is G-S1's three checks, the synthetic counts and the decoy's
    among them, since those are the gate.
    """
    def walk(node, path):
        if isinstance(node, dict):
            for k, v in node.items():
                if path == '$' and k == 'spec_verbatim':
                    continue
                assert str(k) not in BANNED_IN_A_FAIL, \
                    f'{path}.{k} is not a gate value'
                walk(v, f'{path}.{k}')
        elif isinstance(node, list):
            for i, v in enumerate(node):
                walk(v, f'{path}[{i}]')

    walk(payload, '$')


def _write_schema(payload: dict, out_dir: Path,
                  prefix: str) -> tuple[Path, str]:
    """S9's writer under this stage's name: a dated file, never an overwrite."""
    return dw._write_s9(payload, out_dir, prefix)


def schema_meta(head: str, spec_rev: str, spec_sha256: str, porcelain: list,
                variant: str, verbatim: dict, constants: dict,
                sources: dict) -> dict:
    """The meta every file of this stage carries, the fail file's included."""
    return {'stage': 'schema_judge', 'head': head, 'spec_rev': spec_rev,
            'spec_sha256': spec_sha256,
            'registration': SCHEMA_REGISTRATION,
            'tree_dirty': any(not ln.startswith('??') for ln in porcelain),
            'untracked': sum(1 for ln in porcelain if ln.startswith('??')),
            'variant': variant, 'spec': SCHEMA_SPEC_NAME,
            'spec_verbatim': verbatim, 'constants': constants,
            'sources': sources,
            'note': '§2-§7 come from schema_walls (no truth, D-S5); the 20 '
                    'maps from divergence_walls.load_side; truth from '
                    'graph_walls.sdf_wall_faces; A9 from '
                    'graph_walls.validate_map. Every map matched to truth goes '
                    'through SchemaDoor, which raises until G-S1 holds.'}


def _print_gate(gate: dict) -> None:
    print('G-S1: ' + '  '.join(
        f'check {n.split("_")[0]} '
        f'{"HOLD" if ok else ("FAIL" if ok is False else "-")}'
        for n, ok in gate['parts'].items())
        + f'   {"HOLD" if gate["all_ok"] else "FAIL"}')
    c1 = gate['1_synthetic']
    if c1 is not None:
        for part in ('the_160_grids', 'the_640_grids'):
            p = c1.get(part)
            if p is None:
                continue
            print(f'  check 1 {part}: {p["n_blocks"]} blocks, '
                  f'{p["n_grids"]} grids, {p["file"]}')
            for rec in p['differing'][:20]:
                print(f'    {rec["what"]}: pinned {rec["pinned"]!r}  stage '
                      f'{rec["stage"]!r}')
    c2 = gate['2_frame_and_truth']
    if c2 is not None:
        print(f'  check 2: A9 recall equal within {c2["tol"]:g} on '
              f'{c2["n_equal"]}/{c2["n"]} maps')
        for m in c2['differing']:
            print(f'    {m["map"]}: stored {m["recall"]["stored"]!r}  stage '
                  f'{m["recall"]["computed"]!r}')
    c3 = gate['3_decoy_X_S1']
    if c3 is not None:
        print(f'  check 3 (X-S1): {c3["n_maps_clean"]}/{c3["n_maps"]} maps '
              f'with no segment matching the world shifted by '
              f'{c3["shift_m"]}')
        for m in c3['differing']:
            print(f'    {m["map"]}: {m["n_matched"]} of {m["n_with_axis_ends"]} '
                  'segments matched the decoy')


def _print_verdicts(verdicts: dict) -> None:
    for name in PREDICTIONS:
        v = verdicts[name]
        print(f'{name}: {"HOLD" if v["holds"] else "FAIL"}  ', end='')
        if 'n_found' in v:
            print(f'{v["n_found"]} of {v["of"]} found, {v["rule"]}')
            for r in v['rows']:
                if r['missed']:
                    print(f'  {r["map"]}: found {r["found"]}, missed '
                          f'{r["missed"]}')
        else:
            print(f'{v["n_maps_with"]} of {v["of"]} maps, {v["rule"]} '
                  f'({v["what"]})')
            for m in v['maps']:
                print(f'  {m}')


# ──────────────────────────── the stage, in order ────────────────────────────

def map_thresholds(sides: dict, constants: dict) -> dict:
    """Every map's rho, eps and g against the values found in the spec.

    A refusal, not a gate check: G-S1's three checks are about the method, and
    this is about the corpus the method is applied to.
    """
    rho_found = constants['rho']['maps']
    out = []
    for stem in sorted(sides):
        th = sides[stem]['graph']['thresholds']
        rho, eps, g = (th['rho']['value'], th['eps']['value'],
                       th['g']['value'])
        params = gw.Params(g=g)
        if rho != rho_found:
            die(f'{stem}: rho {rho!r} is not the {rho_found!r} §9 states')
        if rho != sides[stem]['rho']:
            die(f'{stem}: the JSON\'s rho {rho!r} is not the map\'s '
                f'{sides[stem]["rho"]!r}')
        if eps != params.eps(rho):
            die(f'{stem}: eps {eps!r} is not Params.eps(rho) '
                f'{params.eps(rho)!r}')
        if g != constants['g']['value_m']:
            die(f'{stem}: g {g!r} is not the {constants["g"]["value_m"]!r} §4 '
                'states')
        checks = {
            'tau_c': (sw.tau_c(eps, rho), constants['tau_c']['value_m']),
            'eps + rho/2': (eps + rho / 2.0,
                            constants['eps']['eps_plus_half_rho_m']),
            'tau_e': (sw.tau_e(rho), constants['tau_e']['value_m']),
            'tau_m': (TAU_M_CELLS * rho, constants['tau_m']['value_m']),
            'tau_end': (TAU_END_CELLS * rho, constants['tau_end']['value_m']),
        }
        for name, (got, want) in checks.items():
            if round(got, 4) != round(want, 4):
                die(f'{stem}: {name} is {got!r} at this map\'s rho, and the '
                    f'spec states {want!r}')
        out.append({'map': stem, 'rho': rho, 'eps': eps, 'g': g,
                    **{k: v[0] for k, v in checks.items()},
                    's_max': sw.S_MAX_M})
    rhos = {r['rho'] for r in out}
    if len(rhos) != 1:
        die(f'the 20 maps carry {sorted(rhos)} as rho; §9 states one value')
    return {'maps': out, 'rho': rhos.pop(),
            'tau_m': out[0]['tau_m'], 'tau_end': out[0]['tau_end']}


def write_per_map(sides: dict, out_dir: Path, graph_walls_commit: str
                  ) -> tuple[dict, list]:
    """§8's per-map files, written, read back and hashed.

    From here on §9 sees only what the files hold (D-S5), so the records the
    judging reads are the ones read back from disk, not the ones in memory.
    """
    records, files = {}, []
    for stem in sorted(sides):
        rec = sw.schema_for_map(sides[stem], graph_walls_commit)
        path, sha = sw.write_record(rec, out_dir)
        back = json.loads(path.read_text())
        if back != json.loads(json.dumps(rec)):
            die(f'{path} does not read back as it was written')
        records[stem] = back
        files.append({'map': stem, 'file': path.name, 'sha256': sha,
                      'counts': back['counts']})
    return records, files


def schema_stage(sides: dict, spawn: list, world: dict, stored: dict,
                 constants: dict, meta: dict, check1_of, a9_of,
                 out_dir: Path, run_dir: Path, graph_walls_commit: str,
                 t0: float | None = None) -> dict:
    """G-S1, then §8's files, then §9 on the 20 maps and §10's verdicts.

    The order, each step only after everything before it held:
    1. every map's rho, eps and g against the found values (a refusal);
    2. check 1, the pinned synthetic counts, on 800 extractions;
    3. check 2, A9 recomputed from each map's segments, against the stored run;
    4. the per-map files, written and read back;
    5. check 3, X-S1's decoy, on the records the files hold;
    6. the door opens, and the 20 maps are matched to truth;
    7. the verdicts, then the reported items.

    On a failure the stage writes the gate's results and stops at exit 2; the
    payload is walked for a match on a real map or a verdict first. Reads no
    corpus file and resolves no spawn; `run_schema` is this function plus the I/O.
    """
    t0 = time.monotonic() if t0 is None else t0
    stems = sorted(sides)
    th = map_thresholds(sides, constants)
    tol = {'tau_m': th['tau_m'], 'tau_end': th['tau_end']}
    now = truth(world)
    check_truth(now, constants)
    decoy = truth(world, constants['decoy shift']['m'])

    c1 = check1_of()
    c2 = c3 = None
    records: dict = {}
    files: list = []
    if c1 is not None and c1['ok']:
        c2 = check2_frame_and_truth(stems, stored, a9_of,
                                    constants['1e-9']['gate'],
                                    stored_step(stored))
    if c2 is not None and c2['ok']:
        records, files = write_per_map(sides, run_dir, graph_walls_commit)
        c3 = check3_decoy(records, spawn, decoy, tol)

    gate = {'1_synthetic': c1, '2_frame_and_truth': c2, '3_decoy_X_S1': c3}
    parts = {n: (None if c is None else c['ok']) for n, c in gate.items()}
    gate['parts'] = parts
    gate['all_ok'] = all(v is True for v in parts.values())
    door = SchemaDoor(records, spawn, now, tol)
    _print_gate(gate)

    if not door.open_after(c1, c2, c3):
        fail = dict(meta)
        fail.update({'stage': 'schema_judge_gate_fail',
                     'seconds': time.monotonic() - t0, 'G-S1': gate,
                     'thresholds': th, 'truth': {'counts': now['counts']},
                     'per_map_files': files, 'n_door_calls': len(door.asked)})
        _no_verdict_in_schema(fail)
        path, _sha = _write_schema(fail, out_dir, 'schema_gate_fail')
        die('G-S1 failed; no map was matched to truth and no verdict was '
            f'computed, written or printed. {path}')

    # ── the door is open: §9 on the 20 maps ────────────────────────────────
    judged = {stem: door.judge(stem) for stem in stems}
    if door.asked != stems:
        die('one door call per map')
    judgments = []
    for stem in stems:
        k, cut = gw._k_and_cut(stem, '§10 needs the robot and the cut')
        judgments.append({'map': stem, 'robot': k, 'cut': cut,
                          **map_counts(judged[stem])})
    verdicts = verdicts_from(judgments, constants, now)
    _print_verdicts(verdicts)

    payload = dict(meta)
    payload.update({
        'seconds': time.monotonic() - t0,
        'rho_m': th['rho'], 'tolerances': tol, 'n_maps': len(stems),
        'truth': {'counts': now['counts'],
                  'faces': now['faces'], 'corners': now['corners'],
                  'slabs': now['slabs'],
                  'min_same_facing_gap': now['min_same_facing_gap']},
        'thresholds': th,
        'per_map_files': files,
        'G-S1': gate,
        'judgments': judgments,
        'verdicts': verdicts,
        'reported': {
            'per_map': [{'map': stem, 'cut': j['cut'],
                         'faces': judged[stem]['faces'],
                         'inside_corners': judged[stem]['inside_corners'],
                         'slabs': judged[stem]['slabs'],
                         'slab_ends': judged[stem]['slab_ends'],
                         'openings': judged[stem]['openings'],
                         'distances_and_faces': judged[stem]['reported']}
                        for stem, j in zip(stems, judgments)],
        },
    })
    path, sha = _write_schema(payload, out_dir, 'schema_judge')
    out = dict(payload)
    out['written'] = {'path': str(path), 'sha256': sha}
    return out


def run_schema(out_dir: Path = OUT_DIR_DEFAULT,
               splice_dir: Path = SPLICE_DIR_DEFAULT,
               variant: str = dw.CORPUS_VARIANT,
               a9_dir: Path = gw.OUT_DIR_DEFAULT) -> dict:
    """The stage, end to end. Spec v0.2, §9 and §10.

    Before anything is loaded:
    1. REFUSES ON AN EDITED TREE, as S5 to S13 do.
    2. REGISTRATION BEFORE RESULT: `5bf3ee2`, the commit that registered §9,
       §10, G-S1 and P-S1 to P-S8, must be an ancestor of HEAD, and the spec at
       HEAD must carry "### Gate G-S1" and "**P-S1," to "**P-S8,".
    3. §1-§10 AT HEAD MUST BE THE REGISTERED TEXT, byte for byte (ruling 4).
       Ancestry alone does not catch a later edit to the registered text.
    4. The registered text is read from the spec AT HEAD by script and stored
       verbatim, and every number the stage applies is found in it.
    5. THE FOUR FILES ARE PINNED BY HASH: G-S1 check 1 is an equality against
       two scratch outputs and the report that cites them, check 2 against one
       A8-A11 run.
    """
    t0 = time.monotonic()
    out_dir, splice_dir, a9_dir = Path(out_dir), Path(splice_dir), Path(a9_dir)

    porcelain = dw._git('status', '--porcelain').splitlines()
    dirty = [ln for ln in porcelain if not ln.startswith('??')]
    if dirty:
        die('the working tree has tracked changes, so a result file would name '
            'a commit it was not produced by:\n  ' + '\n  '.join(dirty))
    head = dw._git('rev-parse', 'HEAD')

    if not dw._is_ancestor(SCHEMA_REGISTRATION):
        die(f'{SCHEMA_REGISTRATION} registered §9, §10, G-S1 and P-S1 to P-S8 '
            f'and is not an ancestor of HEAD {head[:7] or "(unknown)"}; the '
            'registration must be in history behind the code that tests it')
    spec_text = dw._git('show', f'HEAD:{SPEC_REL}')
    for needed in (GATE_HEADING, *(f'**{p},' for p in PREDICTIONS)):
        if needed not in spec_text:
            die(f'the spec at HEAD carries no "{needed}"; this stage evaluates '
                'registered predictions behind a registered gate or it does '
                'not run')
    registered = dw._git('show', f'{SCHEMA_REGISTRATION}:{SPEC_REL}')
    if registered_body(spec_text) != registered_body(registered):
        die(f'§1-§10 at HEAD differ from {SCHEMA_REGISTRATION}; the stage '
            'applies the text that was registered, not an edited one')
    verbatim = schema_verbatim(spec_text)
    constants = schema_constants_from(verbatim)
    spec_rev = dw._git('log', '-1', '--format=%h', '--', SPEC_REL)
    graph_walls_commit = dw._git('log', '-1', '--format=%h', '--',
                                 'experiments/analysis/graph_walls.py')

    sources, pinned = {}, {}
    for name, path, want, what in (
            ('a9', a9_dir / A9_FILE, A9_SHA256,
             "G-S1 check 2 is an equality against its A9"),
            ('synth2_out', splice_dir / SYNTH2_OUT, SYNTH2_SHA256,
             'G-S1 check 1 reproduces its 160-grid counts'),
            ('check_md', splice_dir / CHECK_MD, CHECK_MD_SHA256,
             'G-S1 check 1 names it, and it pins the 160-grid output'),
            ('match_d2_out', splice_dir / MATCH_D2_OUT, MATCH_D2_SHA256,
             'G-S1 check 1 reproduces its 640-grid counts')):
        if not path.is_file():
            die(f'{path} is missing; {what}')
        sha = dw._sha256(path)
        if sha != want:
            die(f'{path.name} has sha256 {sha}, not the {want} G-S1 is stated '
                f'against; that is a different file')
        sources[name] = {'file': path.name, 'sha256': sha}
        pinned[name] = path
    # The spec's sha256 is the worktree file's, which is HEAD's: the tree is
    # clean, and `_git show` strips the trailing newline the file carries.
    sources['spec'] = {'file': SPEC_REL,
                       'sha256': dw._sha256(REPO_ROOT / SPEC_REL)}
    sdf_sha = dw._sha256(REPO_ROOT / WORLD_SDF_REL)
    if not sdf_sha.startswith(constants['files']['sdf_sha256_prefix']):
        die(f'{WORLD_SDF_REL} has sha256 {sdf_sha}, and §1 names '
            f'{constants["files"]["sdf_sha256_prefix"]}…')
    sources['world_sdf'] = {'file': WORLD_SDF_REL, 'sha256': sdf_sha}

    import fit_world_transform as fwt                         # noqa: PLC0415

    spawn = fwt.resolve_spawn_poses(check_spawn_rev(constants))

    stems = dw.corpus_stems(variant)
    pattern = constants['rows']['pattern']
    want_stems = [pattern.replace('{K}', str(k)).replace('{C}', str(c))
                  for k in range(constants['rows']['k'][0],
                                 constants['rows']['k'][1] + 1)
                  for c in constants['rows']['cuts']]
    if sorted(stems) != sorted(want_stems) \
            or len(stems) != constants['rows']['n']:
        die(f'§10 names {len(want_stems)} rows as {pattern} and the corpus is '
            f'{len(stems)} stems of variant {variant}')
    sides = {stem: dw.load_side(stem) for stem in stems}

    world = gw.sdf_wall_faces(gw.L_MIN)
    radius, radius_src = gw.read_robot_radius()
    stored = json.loads(pinned['a9'].read_text())['per_map']
    samples = gw.face_samples(world, stored_step(stored))
    params = gw.Params(g=constants['g']['value_m'])
    sources['robot_radius'] = {'value_m': radius, 'source': radius_src}

    def check1_of() -> dict:
        t = time.monotonic()
        out = check1(splice_dir, params, constants, world, spawn,
                     {'tau_m': TAU_M_CELLS * CHECK_RHO,
                      'tau_end': TAU_END_CELLS * CHECK_RHO})
        print(f'  check 1: {time.monotonic() - t:.1f} s', flush=True)
        return out

    def a9_of(stem: str) -> dict:
        t = time.monotonic()
        out = a9_of_map(sides[stem], spawn, world, samples, radius)
        print(f'  A9 {stem}: {time.monotonic() - t:.1f} s', flush=True)
        return out

    meta = schema_meta(head, spec_rev, sources['spec']['sha256'], porcelain,
                       variant, verbatim, constants, sources)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    return schema_stage(sides, spawn, world, stored, constants, meta,
                        check1_of, a9_of, out_dir,
                        out_dir / f'schema_maps_{stamp}', graph_walls_commit,
                        t0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out-dir', type=Path, default=OUT_DIR_DEFAULT,
                    help='where the results file and the per-map directory go')
    ap.add_argument('--splice-dir', type=Path, default=SPLICE_DIR_DEFAULT,
                    help="G-S1 check 1's three pinned files")
    ap.add_argument('--a9-dir', type=Path, default=gw.OUT_DIR_DEFAULT,
                    help="G-S1 check 2's pinned A8-A11 run")
    args = ap.parse_args()
    run_schema(args.out_dir, args.splice_dir, a9_dir=args.a9_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
