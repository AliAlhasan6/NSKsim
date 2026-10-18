#!/usr/bin/env python3
"""The stage that judges the B2 face graph: G-E1, then P-E1 to P-E4.

docs/specs/SPEC_b2_edges.md v0.2, §5, registered at `358650a`. It follows the
schema stage's pattern (schema_judge.py): before anything is loaded it refuses
an edited tree, requires the registration to be an ancestor of HEAD and §1-§5
at HEAD to be the registered text byte for byte (Q15), stores the registered
text verbatim and finds in it every number it applies. Then G-E1's three items
in the build plan's order (Q8): items 2 and 3 use truth alone and run before
any map is read; item 1 runs on the per-map files once they are written and
read back. Then, and only then, M1 on the 20 maps and P-E1 to P-E4 on the
five at cut1200.

WHERE TRUTH ENTERS (D-E6). Here, and nowhere else in the face graph. §2-§4 are
edges_walls.py, which reads no truth and whose per-map files hold none; this
stage writes those files, reads them back, and from then on judges only what
the files hold. Truth is schema_judge.truth on graph_walls.sdf_wall_faces, the
12 long faces the schema judge matches against, and it is cast through
edges_walls.cast: maps and truth go through one code path.

WHAT THIS FILE OWNS, AND WHAT IT BORROWS. It owns truth as §3.3's faces and
occluders (the 12 faces, each over its exposed extent, and the 8 slab-end
caps), the through-slab count, G-E1's three items, the door, the judging and
the four verdicts. Borrowed, not rewritten:

    edges_walls.cast, facing_edges, components    §3.3 and §4, maps and truth
    edges_walls.read_inputs, join, edges_for_map  the inputs and §4's file
    edges_walls.write_record
    schema_judge.truth, match_faces (M1)          the schema stage's truth and M1
    schema_judge.spec_section, spec_item, ...     the registered-text readers,
                                                  refusing as schema_judge (Q28)
    schema_judge.schema_verbatim, ...constants    tau_m, the spawn rev, the SDF
                                                  prefix, the truth counts (Q16)
    divergence_walls._git, _sha256, _is_ancestor  provenance
    divergence_walls._write_s9, corpus_stems      the writer and the 20 rows

THE PORT IS THE SCRIPT'S. Truth's faces and the through-slab test are ported
line by line from experiments/logs/splice/edges_truth_fstar.py (`true_faces`,
`through_slab`), which computed F* under v0.1 (`6dbc3354…`). Without the caps,
the stage's cast reproduces that file's 12 rows to its printed decimals, or it
refuses (Q14, Q34).

INPUTS (Q6). Schema run 1's 20 per-map files, the files G-S1 passed on, each
checked against the sha256 that run's pinned results file (`84ffdb5a…`) gives
it; and the graph_walls files in experiments/logs/graph_walls/, joined to them
strictly (Q7). Nothing is extracted.

IMPORTS. edges_walls and schema_judge at module scope, and through them
schema_walls, graph_walls and divergence_walls; `fit_world_transform`, which
shells out to git for the spawn table, inside `run_edges`, as run_schema does.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'experiments' / 'analysis'))

import edges_walls as ew                                      # noqa: E402
import schema_judge as sj                                     # noqa: E402

dw, gw = sj.dw, sj.gw

OUT_DIR_DEFAULT = gw.LOGS_DIR / 'edges'
SPLICE_DIR_DEFAULT = gw.LOGS_DIR / 'splice'
SPEC_REL = 'docs/specs/SPEC_b2_edges.md'
EW_REL = 'experiments/analysis/edges_walls.py'

# ── the registration ─────────────────────────────────────────────────────────
EDGES_REGISTRATION = '358650a'     # registered §5: F*, G-E1 and P-E1 to P-E4
EDGES_SPEC_NAME = 'SPEC_b2_edges v0.2, §5 (F*, G-E1, P-E1-P-E4)'

# Q15: §1-§5 at HEAD must be byte-identical to the registered text. §6 (Open
# items) is left out, so closing an open item does not block a rerun.
REGISTERED_FROM = '## 1. Decisions'
REGISTERED_TO = '## 6. Open items'

S1_HEADING = '## 1. Decisions (agreed 18 Oct 2026)'
S33_HEADING = '### 3.3 `facing`'
S34_HEADING = '### 3.4 Numeric conventions'
S51_HEADING = '### 5.1 F*, pinned'
GATE_HEADING = '### 5.2 Gate G-E1'
S53_PREFIX = '### 5.3 Predictions'
PREDICTION_LABELS = {
    'P-E1': '**P-E1. One component.**',
    'P-E2': '**P-E2. F* recovered.**',
    'P-E3': '**P-E3. No wrong facing edges.**',
    'P-E4': '**P-E4. No unmatched facing edges.**',
}
PREDICTIONS = tuple(sorted(PREDICTION_LABELS))
FSTAR_TABLE_HEAD = ['pair', 'width under v0.1 (m)']

# The build plan's §4.2, one finder per row. There is no manifest in this spec
# to compare against, so a test holds the stage to exactly this list.
FINDERS = ('rho', 'w_min', 'min_samples', 'slack', 'parallel', 'boundary_m',
           'caps', 'fstar', 'fstar_file', 'drop', 'through_slab', 'components',
           'gate_maps', 'gate_through_slab', 'p-e1', 'p-e2', 'p-e3', 'p-e4',
           'rows', 'reported_cuts', 'd_e1')

# ── the pinned files (Q6, Q14) ───────────────────────────────────────────────
SCHEMA_RUN_DIR = gw.LOGS_DIR / 'schema' / 'judge_5a72769_20261018T103953Z'
SCHEMA_RESULTS_DEFAULT = SCHEMA_RUN_DIR / 'schema_judge_20261018T104734Z.json'
SCHEMA_MAPS_DEFAULT = SCHEMA_RUN_DIR / 'schema_maps_20261018T104009Z'
SCHEMA_RESULTS_SHA256 = ('84ffdb5a5d6a75ee9e50bb27658df1878353ea0227dfe6eac98e'
                         'cc3ae5379f18')
FSTAR_OUT = 'edges_truth_fstar_out.txt'
FSTAR_SHA256 = ('6dbc3354a8cd7f5bc2912577f708d3814b76a009142de3db5067ce9d9c7e'
                '6c6a')

# ── registered in 358650a; the finder checks each against its text ──────────
TRUTH_RHO = 0.1          # Q2: truth is cast at the rho the spec writes
BOUNDS = {
    'p-e1': {'components': 1, 'min': 4, 'of': 5},
    'p-e2': {'min_pairs': 11, 'of_pairs': 12, 'min': 4, 'of': 5},
    'p-e3': {'at_most': 1, 'of': 5},
    'p-e4': {'at_most': 1, 'of': 5},
}

# ── Ali's ruling on Q13 (18 Oct 2026), not registered in the spec ───────────
# Inside the arena any face sees the wall opposite it, so "far from every
# wall" is met outside it: a 5 m face at x = +20, facing the arena.
DECOY = {'name': 'decoy', 'axis': '-x', 'q0': [20.0, -2.5], 'q1': [20.0, 2.5]}

NUMBER_WORDS = {'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5,
                'six': 6}
SUP_MINUS = '⁻'
SUPERSCRIPT = '⁰¹²³⁴⁵⁶⁷⁸⁹'


class EdgesGateClosed(RuntimeError):
    """A map's judging asked for before G-E1 allowed it."""


def die(msg: str) -> None:
    """Refuse, at exit 2. The callers' convention in this directory."""
    print(f'edges_judge: {msg}', file=sys.stderr)
    sys.exit(2)


# ─────────────────────── reading the registered text ─────────────────────────

def spec_table(section: str) -> dict:
    """The one Markdown table in a section: its header and its rows of cells.

    New for §5.1 (Q29). The table's lines must be contiguous and every row as
    wide as the header, or the stage refuses.
    """
    lines = section.splitlines()
    at = [i for i, ln in enumerate(lines) if ln.startswith('|')]
    if len(at) < 3 or at != list(range(at[0], at[0] + len(at))):
        die('the section does not hold one contiguous table')
    cells = [[c.strip() for c in lines[i].strip().strip('|').split('|')]
             for i in at]
    head, rule, body = cells[0], cells[1], cells[2:]
    if len(rule) != len(head) or not all(c and set(c) <= set('-:')
                                         for c in rule):
        die(f'the table under {head!r} has no rule line')
    for row in body:
        if len(row) != len(head):
            die(f'a table row {row!r} is not {len(head)} cells wide')
    return {'head': head, 'rows': body}


def spec_nth_item(section: str, n: int) -> str:
    """Numbered item n of a section, through its continuation lines.

    New (Q29): G-E1's items carry no bold label for `spec_item` to find. It
    stops where `spec_item` does, and joins the lines with one space.
    """
    marker = f'{n}. '
    lines = section.splitlines()
    starts = [i for i, ln in enumerate(lines) if ln.startswith(marker)]
    if len(starts) != 1:
        die(f'the section has {len(starts)} items numbered {n}, not one')
    out = [lines[starts[0]][len(marker):].strip()]
    for ln in lines[starts[0] + 1:]:
        s = ln.strip()
        if not s or sj.ITEM_START.match(s) or s.startswith('#'):
            break
        out.append(s)
    return ' '.join(out)


def _heading(text: str, prefix: str) -> str:
    hits = [ln.rstrip() for ln in text.splitlines() if ln.startswith(prefix)]
    if len(hits) != 1:
        die(f'the spec has {len(hits)} headings starting {prefix!r}, not one')
    return hits[0]


def registered_body(text: str) -> str:
    """§1 to §5, the text Q15 compares between HEAD and the registration."""
    i = text.find(REGISTERED_FROM)
    j = text.find(REGISTERED_TO, i + 1) if i >= 0 else -1
    if i < 0 or j < 0:
        die(f'the spec has no §1-§5 body ({REGISTERED_FROM!r} to '
            f'{REGISTERED_TO!r})')
    return text[i:j]


def edges_verbatim(text: str) -> dict:
    """The registered text, read from the spec by script and never retyped.

    One item at a time, so a number cannot be read from a neighbour. §3.3's
    Occluders paragraph is followed by a sub-list that `spec_item` would stop
    at, so it is read as a block up to "**Samples.**".
    """
    s1 = sj.spec_section(text, S1_HEADING)
    s33 = sj.spec_section(text, S33_HEADING)
    s34 = sj.spec_section(text, S34_HEADING)
    s51 = sj.spec_section(text, S51_HEADING)
    s52 = sj.spec_section(text, GATE_HEADING)
    h53 = _heading(text, S53_PREFIX)
    s53 = sj.spec_section(text, h53)
    items = {
        'status': sj.spec_item(text, 'Status:'),
        'D-E1': sj.spec_item(s1, '**D-E1. Nodes are faces.**'),
        'occluders': ' '.join(sj.spec_block(s33, '**Occluders.**',
                                            '**Samples.**').split()),
        'samples': sj.spec_item(s33, '**Samples.**'),
        'edge_rule': sj.spec_item(s33, '**A `facing` edge joins'),
        'C1': sj.spec_item(s34, '**C1.**'),
        'C2': sj.spec_item(s34, '**C2.**'),
        'C7': sj.spec_item(s34, '**C7.**'),
        'fstar_intro': sj.spec_item(s51, 'Under v0.1,'),
        'fstar_is': sj.spec_item(s51, '**F* is these'),
        'components': sj.spec_item(s51, "With truth's slab and corner edges"),
        'gate_1': spec_nth_item(s52, 1),
        'gate_2': spec_nth_item(s52, 2),
        'gate_3': spec_nth_item(s52, 3),
        'rows': h53,
        'match': sj.spec_item(s53, 'A map face is matched'),
        'reported': sj.spec_item(s53, '**Reported, not judged:**'),
    }
    for name, label in PREDICTION_LABELS.items():
        items[name] = sj.spec_item(s53, label)
    return {'blocks': {'5.1': s51, '5.2': s52, '5.3': s53},
            'table': spec_table(s51), 'items': items}


def _power(sup: str) -> float:
    """10^-n from the spec's superscript digits, parsed as text so that it is
    the double nearest 1e-n, which is what the code's own literal is."""
    return float('1e-' + ''.join(str(SUPERSCRIPT.index(c)) for c in sup))


def _word(word: str, what: str) -> int:
    if word not in NUMBER_WORDS:
        die(f'{what}: "{word}" is not a number the stage reads')
    return NUMBER_WORDS[word]


def fstar_rows(table: dict) -> list[list]:
    """§5.1's table as [S_i, S_j, width as written], one row per F* pair.

    The two names are separated by " – " (U+2013); the width has two
    decimals. Any other shape, a repeated pair or a pair written backwards
    refuses.
    """
    if table['head'] != FSTAR_TABLE_HEAD:
        die(f'§5.1\'s table is headed {table["head"]!r}, not '
            f'{FSTAR_TABLE_HEAD!r}')
    out = []
    for pair, width in table['rows']:
        m = re.fullmatch(r'(\S+) – (\S+)', pair)
        if not m or not re.fullmatch(r'\d+\.\d\d', width):
            die(f'§5.1\'s table has a row the stage cannot read: '
                f'{[pair, width]!r}')
        if not m.group(1) < m.group(2):
            die(f'§5.1 writes the pair {pair!r} with the higher name first')
        out.append([m.group(1), m.group(2), width])
    if len({(a, b) for a, b, _w in out}) != len(out):
        die('§5.1\'s table lists a pair twice')
    return out


def fstar_samples(constants: dict) -> dict:
    """F*'s v0.1 widths in whole samples at truth's rho: (S_i, S_j) -> n."""
    rho = constants['rho']['truth']
    return {(a, b): round(2.0 * float(w) / rho)
            for a, b, w in constants['fstar']['pairs']}


def edges_constants_from(verbatim: dict) -> dict:
    """Every number the stage applies, found in the registered text or refused.

    Each finder reads one item, and every number is checked against something
    else: the code's own constant, the same number stated a second time, or
    an identity the text implies (the 20 samples are the four pairs' 0.25 m,
    5 samples each). The numbers that need the world -- 19.9 m, the 8 caps,
    the 12 faces, the 1276 samples and the v0.1 widths -- are checked against
    it in `check_edges_truth`, and D-E1's 5 of 179 against the corpus in
    `corpus_checks`.
    """
    it = {k: ' '.join(v.split()) for k, v in verbatim['items'].items()}
    found: dict = {}

    # §3.3, the edge rule: rho, w_min and the 4 samples, in one sentence.
    n_min, rho, w_min = sj._one(
        r'\|V_ij\| \+ \|V_ji\| ≥ 2·w_min/ρ\*\*, that is, ≥ (\d+) samples at '
        r'ρ = ([0-9.]+), w_min = ([0-9.]+)\.$', it['edge_rule'],
        "§3.3's edge rule")
    n_min, rho, w_min = int(n_min), float(rho), float(w_min)
    found['rho'] = {'truth': rho}
    found['w_min'] = {'value_m': w_min}
    found['min_samples'] = {'n': n_min, 'formula': '2*w_min/rho'}

    # 10⁻⁹ twice, §3.3's sample count and C1's slack; 10⁻¹² in C2.
    (e_samples,) = sj._one(
        rf'⌊\(L\(S_i\) \+ 10{SUP_MINUS}([{SUPERSCRIPT}]+)\)/ρ⌋',
        it['samples'], "§3.3's sample count")
    (e_c1,) = sj._one(rf'a slack of 10{SUP_MINUS}([{SUPERSCRIPT}]+) m at each '
                      r'end', it['C1'], "C1's slack")
    if e_samples != e_c1:
        die(f'§3.3 counts samples with 10^-{e_samples} and C1 closes with '
            f'10^-{e_c1}')
    found['slack'] = {'value_m': _power(e_c1)}
    (e_c2,) = sj._one(rf'\(\|d × e\| ≤ 10{SUP_MINUS}([{SUPERSCRIPT}]+) \|e\|\) '
                      r'is never crossed', it['C2'], "C2's parallel bound")
    found['parallel'] = {'value': _power(e_c2)}
    (bnd,) = sj._one(r"a boundary face's extent is the judge's exposed "
                     r'([0-9.]+) m\.$', it['C7'], 'C7')
    found['boundary_m'] = {'value_m': float(bnd)}
    (caps,) = sj._one(r'On truth these are the (\d+) slab end caps\.',
                      it['occluders'], "§3.3's caps")
    found['caps'] = {'n': int(caps)}

    # §5.1: the 12 faces, F*, and the file it came from, cited twice.
    n_faces, n_pairs, out_file, out_sha = sj._one(
        r"on the judge's (\d+) true faces gave (\d+) facing pairs "
        r'\(`([^`]+)`, sha256 `([0-9a-f]+)…`\):$', it['fstar_intro'], '§5.1')
    (n_is,) = sj._one(r'^\*\*F\* is these (\d+) pairs\.\*\*', it['fstar_is'],
                      'F*')
    pairs = fstar_rows(verbatim['table'])
    if not int(n_pairs) == int(n_is) == len(pairs):
        die(f'§5.1 names {n_pairs} and {n_is} pairs and its table has '
            f'{len(pairs)} rows')
    found['fstar'] = {'n_faces': int(n_faces), 'n_pairs': len(pairs),
                      'pairs': pairs}
    cited = (out_file, out_sha)
    again = sj._one(r'\(`([^`]+)`, sha256 `([0-9a-f]+)…`\)', it['status'],
                    "the status's F* file")
    if cited != again:
        die(f'§5.1 cites {cited} and the status paragraph {again}')
    if not FSTAR_SHA256.startswith(out_sha):
        die(f'the stage pins {FSTAR_OUT} at {FSTAR_SHA256}, and the spec names '
            f'{out_sha}…')
    found['fstar_file'] = {'file': out_file, 'sha256_prefix': out_sha,
                           'cited': 2}

    # The four pairs, 0.25 m and the 20 samples.
    (n_through,) = sj._one(r'remove the (\d+) samples that ran through a slab',
                           it['fstar_is'], "§5.1's through-slab samples")
    word, drop_m, listed = sj._one(
        r'so (\w+) widths drop by ([0-9.]+) m: (.+?)\. No pair is created or '
        r'lost\.', it['fstar_is'], "§5.1's four widths")
    drop_pairs = []
    for s in re.split(r', | and ', listed):
        m = re.fullmatch(r'(\S+) – (\S+)', s)
        if not m:
            die(f'§5.1 names a dropped pair the stage cannot read: {s!r}')
        drop_pairs.append([m.group(1), m.group(2)])
    n_drop = _word(word, "§5.1's dropped widths")
    if n_drop != len(drop_pairs):
        die(f'§5.1 says {word} widths drop and names {len(drop_pairs)} pairs')
    in_table = {(a, b) for a, b, _w in pairs}
    for p in drop_pairs:
        if tuple(p) not in in_table:
            die(f'§5.1 drops the width of {p}, which is not a row of F*')
    per_pair = 2.0 * float(drop_m) / rho
    if abs(per_pair - round(per_pair)) > 1e-9:
        die(f'{drop_m} m is not a whole number of samples at rho {rho}')
    if int(n_through) != n_drop * round(per_pair):
        die(f'§5.1 removes {n_through} samples, and {n_drop} widths each '
            f'{drop_m} m shorter are {n_drop * round(per_pair)}')
    found['drop'] = {'n_pairs': n_drop, 'm': float(drop_m),
                     'samples': round(per_pair), 'pairs': drop_pairs}
    found['through_slab'] = {'n': int(n_through)}

    # Components 1 and 2: reported, not gated (Q12).
    n_comp_faces, with_c = sj._one(r'the (\d+) faces form (\w+) component\.',
                                   it['components'], "§5.1's components")
    n_corners, without_c = sj._one(r'Without the (\d+) inside corners they '
                                   r'form (\w+):', it['components'],
                                   "§5.1's components without corners")
    if int(n_comp_faces) != int(n_faces):
        die(f'§5.1 names {n_faces} true faces and {n_comp_faces} in a '
            'component')
    found['components'] = {
        'with_corners': _word(with_c, "§5.1's components"),
        'without_corners': _word(without_c, "§5.1's components"),
        'n_corners': int(n_corners), 'n_faces': int(n_comp_faces)}

    # G-E1.
    n_g, of_g = sj._one(r'^On (\d+) of (\d+) maps, `corner` edges = inside '
                        r'corners and `slab` edges = pairs, as in the schema '
                        r'file\.$', it['gate_1'], 'G-E1 item 1')
    if not int(n_g) == int(of_g) == len(dw.corpus_stems()):
        die(f'G-E1 item 1 reads {n_g} of {of_g} maps, and the corpus is '
            f'{len(dw.corpus_stems())}')
    found['gate_maps'] = {'n': int(n_g), 'of': int(of_g)}
    g2_n, g2_word, g2_drop, g2_through = sj._one(
        r'^Under v0\.2, truth gives exactly the (\d+) pairs of F\*, the (\w+) '
        r'widths reduced by ([0-9.]+) m, and (\d+) samples that run through a '
        r'slab\.$', it['gate_2'], 'G-E1 item 2')
    if (int(g2_n), _word(g2_word, 'G-E1 item 2'), float(g2_drop)) != \
            (len(pairs), n_drop, float(drop_m)):
        die(f'G-E1 item 2 reads {g2_n} pairs, {g2_word} widths and {g2_drop} m; '
            f'§5.1 gives {len(pairs)}, {word} and {drop_m}')
    found['gate_through_slab'] = {'n': int(g2_through)}
    sj._one(r'^(A decoy face placed far from every wall gets no `facing` '
            r'edge\.)$', it['gate_3'], 'G-E1 item 3')

    # §5.3: the rows, the cuts reported, and the four bounds.
    cut, n_rows = sj._one(r'^### 5\.3 Predictions, at cut(\d+) \((\d+) maps\)$',
                          it['rows'], "§5.3's rows")
    if (int(cut), int(n_rows)) != (dw.FULL_CUT, dw.N_ROBOTS):
        die(f'§5.3 judges cut{cut} on {n_rows} maps, and the corpus has '
            f'cut{dw.FULL_CUT} on {dw.N_ROBOTS}')
    found['rows'] = {'cut': int(cut), 'n': int(n_rows)}
    if '(`match_faces`)' not in it['match']:
        die(f'§5.3 matches by something other than M1: {it["match"]!r}')
    (cuts,) = sj._one(r'F\* recovery at cuts ((?:\d+, )*\d+ and \d+) \(',
                      it['reported'], "§5.3's reported cuts")
    reported = [int(c) for c in re.findall(r'\d+', cuts)]
    if reported != [c for c in dw.CUTS if c != dw.FULL_CUT]:
        die(f'§5.3 reports cuts {reported} and the corpus has {dw.CUTS}')
    found['reported_cuts'] = reported
    for name in PREDICTIONS:
        for other in PREDICTIONS:
            if other != name and other in it[name]:
                die(f"{name}'s item names {other}: {it[name]!r}")
    comp, p1_min, p1_of = sj._one(
        r"Each map's graph is (\w+) connected component\. Bound: ≥ (\d+) of "
        r'(\d+) maps\.$', it['P-E1'], 'P-E1')
    found['p-e1'] = {'components': _word(comp, 'P-E1'), 'min': int(p1_min),
                     'of': int(p1_of)}
    p2 = sj._one(r'Bound: ≥ (\d+) of (\d+), on ≥ (\d+) of (\d+) maps\.$',
                 it['P-E2'], 'P-E2')
    found['p-e2'] = dict(zip(('min_pairs', 'of_pairs', 'min', 'of'),
                             (int(v) for v in p2)))
    (p3,) = sj._one(r'Bound: wrong edges on ≤ (\d+) map\.$', it['P-E3'],
                    'P-E3')
    found['p-e3'] = {'at_most': int(p3), 'of': int(n_rows)}
    (p4,) = sj._one(r'Bound: on ≤ (\d+) map\.$', it['P-E4'], 'P-E4')
    found['p-e4'] = {'at_most': int(p4), 'of': int(n_rows)}
    for name in ('p-e1', 'p-e2'):
        if found[name]['of'] != int(n_rows):
            die(f'{name.upper()} is over {found[name]["of"]} maps and §5.3 '
                f'over {n_rows}')
    if found['p-e2']['of_pairs'] != len(pairs):
        die(f'P-E2 is over {found["p-e2"]["of_pairs"]} pairs and F* has '
            f'{len(pairs)}')

    # D-E1's count, checked against the corpus once it is loaded (Q19).
    n_nam, n_seg, n_maps = sj._one(
        r'`no_axis_member` \((\d+) of (\d+) on the (\d+) maps\)', it['D-E1'],
        'D-E1')
    if int(n_maps) != len(dw.corpus_stems()):
        die(f'D-E1 counts over {n_maps} maps and the corpus is '
            f'{len(dw.corpus_stems())}')
    found['d_e1'] = {'no_axis_member': int(n_nam), 'segments': int(n_seg),
                     'maps': int(n_maps)}

    # The constants the code holds equal the found values.
    for what, got, want in (
            ('edges_walls.W_MIN_M', ew.W_MIN_M, w_min),
            ('edges_walls.MIN_SAMPLES', ew.MIN_SAMPLES, n_min),
            ('edges_walls.SLACK', ew.SLACK, found['slack']['value_m']),
            ('edges_walls.PARALLEL', ew.PARALLEL, found['parallel']['value']),
            ('TRUTH_RHO', TRUTH_RHO, rho),
            ("the stage's F* file", FSTAR_OUT, out_file)):
        if got != want:
            die(f'{what} is {got!r} and the registered text gives {want!r}')
    if round(2.0 * w_min / rho) != n_min:
        die(f'{n_min} samples is not 2 w_min / rho at rho {rho} (Q25)')
    for name, bound in BOUNDS.items():
        if found[name] != bound:
            die(f'{name.upper()} reads {found[name]} and the stage holds '
                f'{bound}')
    assert set(found) == set(FINDERS), sorted(set(found) ^ set(FINDERS))
    return found


# ──────────────────────────── truth under v0.2 ───────────────────────────────

def _face(key: str, axis: str, n, u, q0, q1) -> dict:
    """One face as `edges_walls.cast` takes it, its span the exact axis's."""
    if ew.AXIS_VEC[axis] != n:
        die(f'{key}: the normal {n} is not the axis {axis}')
    t0, t1 = ew.dot(q0, u), ew.dot(q1, u)
    return {'key': key, 'own': (ew.SEGMENT, key), 'axis': axis, 'n_hat': n,
            'u': u, 'q0': q0, 'q1': q1, 'L': t1 - t0,
            'm': ((q0[0] + q1[0]) / 2.0, (q0[1] + q1[1]) / 2.0),
            'span': (u, t0, t1)}


def truth_faces(T: dict) -> list[dict]:
    """The judge's 12 faces as §3.3's faces, in sorted name order.

    A port of the script's `true_faces`: n is the outward normal, u = (n_y,
    -n_x), q0 the end with the smaller t = p . u, and the axis interval the
    exposed extent the judge built (C7). A face's own segment is keyed
    (0, name), so the tie rule's "lower name" is C3's on truth.
    """
    out = []
    for name in sorted(T['faces']):
        f = T['faces'][name]
        n = (float(f['n'][0]), float(f['n'][1]))
        u = (n[1], -n[0])
        if f['ax'] == 'y':                      # normal along y: the line y = v
            a, b = (f['lo'], f['v']), (f['hi'], f['v'])
        else:                                   # normal along x: the line x = v
            a, b = (f['v'], f['lo']), (f['v'], f['hi'])
        q0, q1 = (a, b) if ew.dot(a, u) <= ew.dot(b, u) else (b, a)
        out.append(_face(name, gw.DIRS[f['axis']][4], n, u, q0, q1))
    return out


def decoy_face(decoy: dict) -> dict:
    """G-E1 item 3's decoy as a face, from {name, axis, q0, q1}."""
    n = ew.AXIS_VEC[decoy['axis']]
    u = (n[1], -n[0])
    a = (float(decoy['q0'][0]), float(decoy['q0'][1]))
    b = (float(decoy['q1'][0]), float(decoy['q1'][1]))
    q0, q1 = (a, b) if ew.dot(a, u) <= ew.dot(b, u) else (b, a)
    return _face(decoy['name'], decoy['axis'], n, u, q0, q1)


def truth_occluders(T: dict, faces: list[dict], caps: bool = True) -> list:
    """§3.3's O on truth: each face's own extent, then the 8 slab end caps.

    Faces are keyed (0, name). A cap is the segment between a slab end's two
    outside corners, `schema_judge.truth`'s `outside`, keyed (1, slab, end)
    after every face (C3, Q27). `caps=False` is v0.1's O, which the pinned
    F* file was cast against.
    """
    out = [{'key': [ew.SEGMENT, f['key']], 'kind': 'segment',
            'p0': [f['q0'][0], f['q0'][1]], 'p1': [f['q1'][0], f['q1'][1]]}
           for f in faces]
    if caps:
        for s in sorted(T['slabs'], key=lambda z: z['name']):
            for end in (0, 1):
                a, b = s['outside'][end]
                out.append({'key': [ew.CAP, s['name'], end], 'kind': 'cap',
                            'p0': [float(a[0]), float(a[1])],
                            'p1': [float(b[0]), float(b[1])]})
    return out


def through_slab(p, d, lam_hit: float, rects: list) -> str | None:
    """The first slab whose closed rectangle the open ray 0 < lambda < lam_hit
    meets. The script's `through_slab`, unchanged: G-E1 item 2 requires none.
    """
    for name, (x0, x1, y0, y1) in rects:
        a, b, inside = ew.SLACK, lam_hit - ew.SLACK, True
        for pc, dc, r0, r1 in ((p[0], d[0], x0, x1), (p[1], d[1], y0, y1)):
            if abs(dc) <= ew.PARALLEL:
                inside = r0 - ew.SLACK <= pc <= r1 + ew.SLACK
            else:
                t0, t1 = sorted(((r0 - pc) / dc, (r1 - pc) / dc))
                a, b = max(a, t0), min(b, t1)
            if not inside:
                break
        if inside and a <= b:
            return name
    return None


def truth_graph(world: dict, rho: float, decoy: dict | None = None,
                caps: bool = True) -> dict:
    """§3.3 on the judge's truth: the 12 faces, the 8 caps, and a decoy if given.

    Every sample that has a first crossing is tested for a path through a
    slab's closed rectangle before it (Q11), keyed (face, partner) when it
    sees and (face, cause) when it is blocked. Components are counted with
    truth's slab and corner edges, without the corners, and over facing
    edges alone, as §5.1 states them.
    """
    T = sj.truth(world)
    faces = truth_faces(T) + ([decoy_face(decoy)] if decoy is not None else [])
    occ = truth_occluders(T, faces, caps)
    rects = [(n, r) for n, r in world['rects'] if n.startswith('maze_')]
    through: dict = {}

    def observe(rec):
        if rec['lam'] is None:
            return
        if through_slab(rec['p'], rec['d'], rec['lam'], rects) is None:
            return
        key = (rec['face'], rec['partner'] if rec['partner'] is not None
               else rec['outcome'])
        through[key] = through.get(key, 0) + 1

    V, tally = ew.cast(faces, occ, rho, observe=observe)
    facing, below = ew.facing_edges(faces, V, rho)
    names = [f['key'] for f in faces]
    slab = [tuple(s['faces']) for s in T['slabs']]
    corner = [tuple(c['faces']) for c in T['corners']]
    fe = [(e['i'], e['j']) for e in facing]

    def n_comp(pairs):
        return len(ew.components(names, pairs))

    return {'truth': T, 'rho': rho, 'caps': caps, 'decoy': decoy,
            'faces': faces, 'occluders': occ, 'V': V, 'tally': tally,
            'facing': facing, 'below_w_min': below,
            'through': sorted([a, b, c] for (a, b), c in through.items()),
            'n_through': sum(through.values()),
            'components': {'all': n_comp(fe + slab + corner),
                           'facing_and_slab': n_comp(fe + slab),
                           'facing': n_comp(fe)}}


def fstar_of(G: dict) -> dict:
    """Truth's facing pairs and their attributes: (S_i, S_j) -> attrs."""
    return {(e['i'], e['j']): dict(e['attrs']) for e in G['facing']}


def v0_1_summary(G01: dict) -> dict:
    return {'pairs': [[e['i'], e['j'], e['attrs']['n_ij'], e['attrs']['n_ji']]
                      for e in G01['facing']],
            'n_through': G01['n_through'], 'through': G01['through']}


def check_edges_truth(G01: dict, constants: dict) -> None:
    """The world against the numbers found in the text, before any gate item.

    G01 is truth cast WITHOUT the caps, which is v0.1's O. It must give
    §5.1's 12 widths in whole samples and §5.1's 20 samples through a slab:
    the stage's truth is the truth F* was computed on. The 12 faces, C7's
    19.9 m, §3.3's 8 caps and the 4 corners are checked here too, and the
    v0.1 widths must account for every truth sample (1276).
    """
    T, fs = G01['truth'], constants['fstar']
    if G01['caps'] or G01['decoy'] is not None:
        die('the v0.1 check is cast without caps and without a decoy')
    if len(T['faces']) != fs['n_faces']:
        die(f'the world gives {len(T["faces"])} faces and §5.1 {fs["n_faces"]}')
    for a, b, _w in fs['pairs']:
        for name in (a, b):
            if name not in T['faces']:
                die(f'§5.1 names {name!r}, which is not one of the judge\'s '
                    'faces')
    for name in sorted(sj.BOUNDARY_FACE.values()):
        length = T['faces'][name]['length_m']
        if abs(length - constants['boundary_m']['value_m']) > ew.SLACK:
            die(f'boundary face {name} is {length} m, and C7 states '
                f'{constants["boundary_m"]["value_m"]} m')
    caps = [o for o in truth_occluders(T, []) if o['kind'] == 'cap']
    if len(caps) != constants['caps']['n']:
        die(f'the world gives {len(caps)} caps and §3.3 '
            f'{constants["caps"]["n"]}')
    if len(T['corners']) != constants['components']['n_corners']:
        die(f'the world gives {len(T["corners"])} inside corners and §5.1 '
            f'{constants["components"]["n_corners"]}')
    want = fstar_samples(constants)
    total = sum(t['samples'] for t in G01['tally'].values())
    if total != sum(want.values()):
        die(f'truth has {total} samples, and §5.1\'s widths account for '
            f'{sum(want.values())}')
    got = {(e['i'], e['j']): e['attrs']['n_ij'] + e['attrs']['n_ji']
           for e in G01['facing']}
    if got != want:
        die(f'without caps the stage gives {got}, not §5.1\'s F* {want}')
    if G01['n_through'] != constants['through_slab']['n']:
        die(f'without caps {G01["n_through"]} samples run through a slab, '
            f'and §5.1 states {constants["through_slab"]["n"]}')


def parse_fstar_out(text: str) -> dict:
    """The pinned F* file's world, FACING PAIRS rows and through-slab rows.

    A line where a row should be that the stage cannot read is a refusal.
    """
    lines = text.splitlines()

    def one(prefix):
        hits = [i for i, ln in enumerate(lines) if ln.startswith(prefix)]
        if len(hits) != 1:
            die(f'{FSTAR_OUT} has {len(hits)} lines starting {prefix!r}, not '
                'one')
        return hits[0]

    m = re.fullmatch(r'  world   (\S+)  sha256 ([0-9a-f]{64})',
                     lines[one('  world   ')])
    if not m:
        die(f'{FSTAR_OUT} names its world in a shape the stage cannot read')
    world = {'file': m.group(1), 'sha256': m.group(2)}
    top, below = one('FACING PAIRS (w_ij >= w_min).'), one('BELOW w_min')
    head = [i for i in range(top, below)
            if lines[i].split()[:2] == ['S_i', 'S_j']]
    if len(head) != 1:
        die(f'{FSTAR_OUT} has no single header row of FACING PAIRS')
    rows = []
    for ln in lines[head[0] + 1:below]:
        tok = ln.split()
        if len(tok) != 10 or not (tok[2].isdigit() and tok[3].isdigit()):
            die(f'{FSTAR_OUT} has a FACING PAIRS row the stage cannot read: '
                f'{ln!r}')
        rows.append({'i': tok[0], 'j': tok[1], 'n_ij': int(tok[2]),
                     'n_ji': int(tok[3]), 'width_m': tok[4], 'gap_m': tok[5],
                     'offset_ij': tok[6], 'offset_ji': tok[7],
                     'bearing_ij': tok[8], 'bearing_ji': tok[9]})
    m_below = re.fullmatch(r'BELOW w_min \(.*\): (.*)', lines[below])
    m_n = re.fullmatch(r'F\*: (\d+) facing pairs', lines[one('F*: ')])
    if not m_below or not m_n:
        die(f'{FSTAR_OUT}: the BELOW w_min or F* line is not the shape the '
            'stage reads')
    t0 = one('    before its hit, as i -> j: count.')
    t1 = one('  files opened in the repo')
    through = []
    for ln in lines[t0 + 1:t1]:
        mt = re.fullmatch(r'    (\S+) -> (\S+): (\d+)', ln)
        if not mt:
            die(f'{FSTAR_OUT} has a through-slab row the stage cannot read: '
                f'{ln!r}')
        through.append([mt.group(1), mt.group(2), int(mt.group(3))])
    return {'world': world, 'rows': rows, 'below': m_below.group(1),
            'n_pairs': int(m_n.group(1)), 'through': through}


def check_fstar_out(parsed: dict, G01: dict, constants: dict,
                    sdf_sha256: str) -> None:
    """Q14 and Q34: the pinned F* file is §5.1's table, and the stage's cast
    without caps reproduces its 12 rows to the printed decimals."""
    if parsed['world'] != {'file': sj.WORLD_SDF_REL, 'sha256': sdf_sha256}:
        die(f'{FSTAR_OUT} was computed on {parsed["world"]}, and the world '
            f'is {sj.WORLD_SDF_REL} at {sdf_sha256}')
    fs = constants['fstar']
    if not parsed['n_pairs'] == len(parsed['rows']) == fs['n_pairs']:
        die(f'{FSTAR_OUT} gives {parsed["n_pairs"]} pairs in '
            f'{len(parsed["rows"])} rows, and §5.1 {fs["n_pairs"]}')
    if parsed['below'] != 'none':
        die(f'{FSTAR_OUT} lists pairs below w_min: {parsed["below"]!r}')
    table = {(a, b): w for a, b, w in fs['pairs']}
    for r in parsed['rows']:
        w = table.get((r['i'], r['j']))
        if w is None or f'{float(r["width_m"]):.2f}' != w:
            die(f'{FSTAR_OUT} row {r["i"]} - {r["j"]} width {r["width_m"]} is '
                f'not §5.1\'s {w}')
    got = fstar_of(G01)
    for r in parsed['rows']:
        a = got.get((r['i'], r['j']))
        if a is None:
            die(f'the cast without caps has no edge {r["i"]} - {r["j"]}')
        mine = (a['n_ij'], a['n_ji'], f'{a["width_m"]:.3f}',
                f'{a["gap_m"]:.4f}', f'{a["offset_m"]:.4f}',
                f'{a["bearing_deg"]:.3f}')
        theirs = (r['n_ij'], r['n_ji'], r['width_m'], r['gap_m'],
                  r['offset_ij'], r['bearing_ij'])
        if mine != theirs:
            die(f'{r["i"]} - {r["j"]}: the cast without caps gives {mine} and '
                f'{FSTAR_OUT} {theirs}')
        if (r['offset_ji'], r['bearing_ji']) != (r['offset_ij'],
                                                 r['bearing_ij']):
            die(f'{FSTAR_OUT} row {r["i"]} - {r["j"]} differs in its two '
                'directions, against A7')
    if parsed['through'] != G01['through']:
        die(f'{FSTAR_OUT} lists {parsed["through"]} through a slab and the '
            f'cast without caps {G01["through"]}')


# ───────────────────────────────── G-E1 ──────────────────────────────────────

def gate_2_truth(G: dict, constants: dict) -> dict:
    """G-E1 item 2: under v0.2, truth gives F* exactly, the four widths 0.25 m
    short, and 0 samples through a slab.

    Widths are compared in whole samples (Q10): the four pairs lose 5 each and
    the other eight are equal. The through-slab count is over every sample
    that has a first crossing (Q11). Blocked, no-hit and tie counts and the
    components are beside the item, not part of it (Q12).
    """
    v01 = fstar_samples(constants)
    drop = {tuple(p) for p in constants['drop']['pairs']}
    want = {p: n - (constants['drop']['samples'] if p in drop else 0)
            for p, n in v01.items()}
    got = fstar_of(G)
    rows = []
    for p in sorted(set(want) | set(got)):
        a = got.get(p)
        n = None if a is None else a['n_ij'] + a['n_ji']
        rows.append({'pair': list(p), 'v0_1_samples': v01.get(p),
                     'expected_samples': want.get(p), 'samples': n,
                     'n_ij': None if a is None else a['n_ij'],
                     'n_ji': None if a is None else a['n_ji'],
                     'width_m': None if a is None else a['width_m'],
                     'gap_m': None if a is None else a['gap_m'],
                     'dropped': p in drop,
                     'equal': n is not None and n == want.get(p)})
    pairs_ok = set(got) == set(want)
    widths_ok = all(r['equal'] for r in rows)
    through_ok = G['n_through'] == constants['gate_through_slab']['n']
    tally = G['tally']
    stated = constants['components']
    return {'ok': bool(pairs_ok and widths_ok and through_ok),
            'rho': G['rho'], 'caps': G['caps'],
            'pairs_equal': pairs_ok, 'widths_equal': widths_ok,
            'n_through_slab': G['n_through'], 'through_slab_ok': through_ok,
            'n_pairs': len(got), 'pairs': rows,
            'missing': [list(p) for p in sorted(set(want) - set(got))],
            'extra': [list(p) for p in sorted(set(got) - set(want))],
            'beside': {
                'samples': sum(t['samples'] for t in tally.values()),
                'blocked': {k: t['blocked'] for k, t in sorted(tally.items())},
                'blocked_by': {k: t['blocked_by']
                               for k, t in sorted(tally.items())},
                'blocked_total': sum(t['blocked'] for t in tally.values()),
                'no_hit_total': sum(t['no_hit'] for t in tally.values()),
                'ties_total': sum(t['ties'] for t in tally.values()),
                'below_w_min': G['below_w_min'], 'through': G['through'],
                'components': G['components'],
                'components_stated': {'all': stated['with_corners'],
                                      'facing_and_slab':
                                          stated['without_corners']}}}


def gate_3_decoy(G3: dict, G2: dict, decoy: dict) -> dict:
    """G-E1 item 3 (Q13): the decoy gets no facing edge, and nothing else
    changes: every other edge, attributes and all, and every face's tally
    are those of item 2's cast."""
    name = decoy['name']
    mine = [e for e in G3['facing'] if name in (e['i'], e['j'])]
    others = [e for e in G3['facing'] if name not in (e['i'], e['j'])]
    same_edges = others == G2['facing']
    same_tally = {k: v for k, v in G3['tally'].items() if k != name} == \
        G2['tally']
    return {'ok': bool(not mine and same_edges and same_tally),
            'decoy': decoy, 'decoy_facing_edges': mine,
            'decoy_below_w_min': [r for r in G3['below_w_min']
                                  if name in (r['i'], r['j'])],
            'decoy_tally': G3['tally'][name],
            'other_edges_unchanged': same_edges,
            'tallies_unchanged': same_tally}


def _schema_structure(schema_rec: dict) -> dict:
    """§3.1's and §3.2's edges as the schema file states them, built here from
    its own fields and not through edges_walls."""
    corner = [(k['segments'][0], k['segments'][1],
               {'c': k['c'], 'near_end': k['near_end'], 'near_m': k['near_m'],
                'states': k['states']})
              for k in schema_rec['inside_corners']]
    slab = [(p['segments'][0], p['segments'][1],
             {'separation_m': p['separation_m'], 'overlap_m': p['overlap_m'],
              'ends': [{f: e[f] for f in ('record', 'diff_m', 'aligned',
                                          'state')} for e in p['ends']]})
            for p in schema_rec['pairs']]
    return {'corner': corner, 'slab': slab}


def _in_order(edges: list) -> list:
    return sorted(edges, key=lambda t: (t[0], t[1],
                                        json.dumps(t[2], sort_keys=True)))


def gate_1_structure(records: dict, schema_recs: dict, constants: dict) -> dict:
    """G-E1 item 1: on every map, `corner` edges = inside corners and `slab`
    edges = pairs, one to one with their attributes (Q9), in the record read
    back from its file against the schema file."""
    maps = []
    for stem in sorted(records):
        want = _schema_structure(schema_recs[stem])
        row = {'map': stem}
        for kind in ('corner', 'slab'):
            got = _in_order([(e['i'], e['j'], e['attrs'])
                             for e in records[stem]['edges']
                             if e['type'] == kind])
            exp = _in_order(want[kind])
            row[kind] = {'schema': len(exp), 'edges': len(got),
                         'equal': len(got) == len(exp)
                         and all(g == x for g, x in zip(got, exp))}
        row['ok'] = row['corner']['equal'] and row['slab']['equal']
        maps.append(row)
    covers = sorted(records) == sorted(schema_recs)
    n_ok = sum(m['ok'] for m in maps)
    n_want = constants['gate_maps']['n']
    return {'ok': bool(covers and n_ok == len(maps) == n_want),
            'covers_the_maps': covers, 'n_maps': len(maps),
            'n_maps_equal': n_ok, 'of': n_want, 'maps': maps,
            'differing': [m for m in maps if not m['ok']]}


class EdgesDoor:
    """The one door through which the stage matches a real map to truth.

    `judge(stem)` raises `EdgesGateClosed` until `open_after` has been handed
    G-E1's three items and all three hold.
    """

    def __init__(self, records: dict, schema_recs: dict, spawn: list,
                 truth_: dict, tau_m, fstar: dict):
        self._records, self._schema = records, schema_recs
        self._spawn, self._truth = spawn, truth_
        self._tau_m, self._fstar = tau_m, fstar
        self._open = False
        self.asked: list = []

    @property
    def is_open(self) -> bool:
        return self._open

    def open_after(self, *items) -> bool:
        self._open = (len(items) == 3 and all(
            c is not None and c.get('ok') is True for c in items))
        return self._open

    def judge(self, stem: str) -> dict:
        if not self._open:
            raise EdgesGateClosed(f'{stem}: G-E1 has not passed; no map is '
                                  'matched to truth')
        self.asked.append(stem)
        k, _cut = gw._k_and_cut(stem, 'the matching needs the robot')
        return judge_edges(self._records[stem], self._schema[stem],
                           self._truth, self._spawn[k], self._tau_m,
                           self._fstar)


# ──────────────────────── §5.3: judging one map ──────────────────────────────

def judge_edges(edge_rec: dict, schema_rec: dict, truth_: dict, shift,
                tau_m: float, fstar: dict) -> dict:
    """M1 and the three kinds of facing edge, for one map's record.

    M1 is `schema_judge.match_faces` on the schema file, carried by M0's
    spawn. A facing edge is an F* edge when its two faces match an F* pair,
    wrong when both match and the pair is not in F*, and unmatched when M1
    leaves either face unmatched. An F* pair counts once per map however many
    edges realise it (Q17); every such edge is listed with its width and gap
    beside truth's under v0.2, as map minus truth (Q18). Components are
    recounted from the file's own edges and must equal its count.

    Widths, gaps and per-face no-hit counts are reported and never judged.
    Near an arena corner a riser can take a wall's end cells, so a map face
    there may be up to about two cells short and the samples of the face
    opposite it that land there have no hit (e24_room_diag.md §4).
    """
    fm = sj.match_faces(schema_rec, truth_, shift, tau_m)
    ids = [n['id'] for n in edge_rec['nodes']]
    if sorted(ids) != sorted(fm):
        die(f'{edge_rec["map"]}: the nodes {sorted(ids)} are not the segments '
            f'with axis ends {sorted(fm)}')
    comps = ew.components(ids, [(e['i'], e['j']) for e in edge_rec['edges']])
    if len(comps) != edge_rec['counts']['components']:
        die(f'{edge_rec["map"]}: the edges give {len(comps)} components and '
            f'the file counts {edge_rec["counts"]["components"]}')
    facing = [e for e in edge_rec['edges'] if e['type'] == 'facing']
    recovered, realised, wrong, unmatched = set(), [], [], []
    for e in facing:
        fa, fb = fm[e['i']], fm[e['j']]
        rec = {'i': e['i'], 'j': e['j'], 'faces': [fa, fb]}
        if fa is None or fb is None:
            unmatched.append(rec)
            continue
        pair = tuple(sorted((fa, fb)))
        if pair not in fstar:
            wrong.append(rec)
            continue
        recovered.add(pair)
        a, t = e['attrs'], fstar[pair]
        realised.append(dict(rec, pair=list(pair), width_m=a['width_m'],
                             gap_m=a['gap_m'], truth_width_m=t['width_m'],
                             truth_gap_m=t['gap_m'],
                             width_minus_truth_m=a['width_m'] - t['width_m'],
                             gap_minus_truth_m=a['gap_m'] - t['gap_m']))
    per_face = [{'id': r['id'], 'face': fm.get(r['id']),
                 'samples': r['samples'], 'blocked': r['blocked'],
                 'blocked_by': r['blocked_by'], 'no_hit': r['no_hit']}
                for r in edge_rec['diagnostics']['per_face']]
    return {'map': edge_rec['map'], 'faces': fm, 'components': len(comps),
            'n_facing': len(facing),
            'recovered_pairs': sorted(list(p) for p in recovered),
            'fstar_edges': realised, 'wrong_edges': wrong,
            'unmatched_edges': unmatched, 'per_face': per_face}


def row_of(stem: str, j: dict) -> dict:
    """The counts §5.3 judges and reports, for one map."""
    k, cut = gw._k_and_cut(stem, '§5.3 needs the robot and the cut')
    return {'map': stem, 'robot': k, 'cut': cut,
            'components': j['components'], 'n_facing': j['n_facing'],
            'n_recovered': len(j['recovered_pairs']),
            'n_wrong': len(j['wrong_edges']),
            'n_unmatched': len(j['unmatched_edges'])}


def split_rows(rows: list, constants: dict) -> dict:
    """The five rows the predictions are judged on, and the rest by cut.

    Cuts 60, 120 and 240 are reported and never judged (§5.3).
    """
    cut = constants['rows']['cut']
    return {'judged': [r for r in rows if r['cut'] == cut],
            'reported': {str(c): [r for r in rows if r['cut'] == c]
                         for c in constants['reported_cuts']}}


def verdicts_from(rows: list, constants: dict) -> dict:
    """P-E1 to P-E4 in integers, on the five cut1200 rows only."""
    full = split_rows(rows, constants)['judged']
    if len(full) != constants['rows']['n']:
        die(f'§5.3 judges {constants["rows"]["n"]} maps at '
            f'cut{constants["rows"]["cut"]} and the rows hold {len(full)}')
    p1, p2, p3, p4 = (constants[k] for k in ('p-e1', 'p-e2', 'p-e3', 'p-e4'))
    for p in (p1, p2, p3, p4):
        if p['of'] != len(full):
            die(f'a bound is over {p["of"]} maps and the rows hold {len(full)}')
    one = [r['map'] for r in full if r['components'] == p1['components']]
    out = {'P-E1': {
        'holds': len(one) >= p1['min'], 'n_holding': len(one), 'of': p1['of'],
        'min_required': p1['min'],
        'rule': f'{p1["components"]} component on at least {p1["min"]} of '
                f'{p1["of"]} maps',
        'maps': [{'map': r['map'], 'components': r['components']}
                 for r in full]}}
    enough = [r['map'] for r in full if r['n_recovered'] >= p2['min_pairs']]
    out['P-E2'] = {
        'holds': len(enough) >= p2['min'], 'n_holding': len(enough),
        'of': p2['of'], 'min_required': p2['min'],
        'min_pairs': p2['min_pairs'], 'of_pairs': p2['of_pairs'],
        'rule': f'at least {p2["min_pairs"]} of {p2["of_pairs"]} F* pairs on '
                f'at least {p2["min"]} of {p2["of"]} maps',
        'maps': [{'map': r['map'], 'n_recovered': r['n_recovered']}
                 for r in full]}
    for name, p, key, what in (('P-E3', p3, 'n_wrong', 'wrong facing edges'),
                               ('P-E4', p4, 'n_unmatched',
                                'unmatched facing edges')):
        with_ = [r['map'] for r in full if r[key] > 0]
        out[name] = {'holds': len(with_) <= p['at_most'],
                     'n_maps_with': len(with_), 'of': p['of'],
                     'at_most': p['at_most'],
                     'rule': f'at most {p["at_most"]} of {p["of"]} maps',
                     'what': what, 'maps': with_}
    return out


# ───────────────────────────── the inputs ────────────────────────────────────

def load_inputs(results_path, maps_dir, graph_dir, stems: list) -> dict:
    """The 20 schema files and graph_walls files, hashed and joined (Q6, Q7).

    Each schema file must have the sha256 the pinned results file gives it,
    each pair of files must join strictly, and the 20 maps must share one rho
    (Q2). Refusals all; nothing is written.
    """
    results = json.loads(Path(results_path).read_bytes())
    per = {r['map']: r for r in results['per_map_files']}
    if sorted(per) != sorted(stems) or len(per) != len(
            results['per_map_files']):
        die(f'the results file lists {sorted(per)}, not the {len(stems)} rows')
    out: dict = {'stems': sorted(stems), 'schema': {}, 'graph': {},
                 'schema_sha': {}, 'graph_sha': {}}
    rhos = {}
    for stem in sorted(stems):
        sp = Path(maps_dir) / per[stem]['file']
        gp = Path(graph_dir) / f'{stem}.json'
        for p in (sp, gp):
            if not p.is_file():
                die(f'{p} is missing')
        schema_rec, graph, s_sha, g_sha = ew.read_inputs(sp, gp)
        if s_sha != per[stem]['sha256']:
            die(f'{sp.name} has sha256 {s_sha}, and the results file gives '
                f'{per[stem]["sha256"]}: not the file G-S1 passed on')
        if schema_rec.get('map') != stem:
            die(f'{sp.name} holds map {schema_rec.get("map")!r}, not {stem}')
        ew.join(schema_rec, graph)
        rhos[stem] = schema_rec['thresholds']['rho']
        out['schema'][stem], out['graph'][stem] = schema_rec, graph
        out['schema_sha'][stem], out['graph_sha'][stem] = s_sha, g_sha
    if len(set(rhos.values())) != 1:
        die(f'the maps carry {sorted(set(rhos.values()))} as rho; they are cast '
            'at one value')
    out['rho'] = next(iter(rhos.values()))
    return out


def corpus_checks(inputs: dict, constants: dict, schema_constants: dict
                  ) -> dict:
    """The corpus against the found values: rho, the 4 samples at it (Q25),
    M1's tau_m (Q16) and D-E1's 5 of 179 (Q19). Refusals, not gate items."""
    rho = inputs['rho']
    if rho != schema_constants['rho']['maps']:
        die(f'the maps carry rho {rho!r}, and SPEC_b2_schema §9 states '
            f'{schema_constants["rho"]["maps"]!r}')
    if round(2.0 * ew.W_MIN_M / rho) != ew.MIN_SAMPLES:
        die(f'{ew.MIN_SAMPLES} samples is not 2 w_min / rho at the maps\' rho '
            f'{rho!r}')
    # As the schema stage applied it: TRUTH_TOL_CELLS cells at the maps' rho.
    tau_m = sj.TAU_M_CELLS * rho
    if round(tau_m, 4) != schema_constants['tau_m']['value_m']:
        die(f'tau_m is {tau_m!r} at the maps\' rho and SPEC_b2_schema states '
            f'{schema_constants["tau_m"]["value_m"]!r}')
    segs = [s for r in inputs['schema'].values() for s in r['segments']]
    got = {'no_axis_member': sum(s['no_axis_ends'] == 'no_axis_member'
                                 for s in segs),
           'segments': len(segs), 'maps': len(inputs['schema'])}
    if got != constants['d_e1']:
        die(f'the corpus has {got}, and D-E1 states {constants["d_e1"]}')
    return {'rho': rho, 'tau_m': tau_m,
            'min_samples_at_rho': round(2.0 * ew.W_MIN_M / rho),
            'd_e1': dict(got, no_axis=sum(s['no_axis_ends'] == 'no_axis'
                                          for s in segs))}


def write_per_map(inputs: dict, run_dir: Path, builder_commit: str
                  ) -> tuple[dict, list]:
    """§4's per-map files, written, read back and hashed.

    From here on the stage judges only what the files hold, so the records
    item 1 and the door read are the ones read back from disk.
    """
    records, files = {}, []
    for stem in inputs['stems']:
        rec = ew.edges_for_map(
            inputs['schema'][stem], inputs['graph'][stem],
            {'schema_file_sha256': inputs['schema_sha'][stem],
             'graph_walls_file_sha256': inputs['graph_sha'][stem],
             'builder_commit': builder_commit})
        path, sha = ew.write_record(rec, run_dir)
        back = json.loads(path.read_text())
        if back != json.loads(json.dumps(rec)):
            die(f'{path} does not read back as it was written')
        records[stem] = back
        files.append({'map': stem, 'file': path.name, 'sha256': sha,
                      'counts': back['counts']})
    return records, files


# ───────────────────── the fail path, the meta and the writer ────────────────

# `at_most`, `min` and `min_pairs` are registered bounds and part of every
# meta's constants; what a fail file may not hold is a verdict or a map's
# matching.
BANNED_IN_A_FAIL = {'judgments', 'verdicts', 'reported', 'holds', 'n_holding',
                    'min_required', 'n_maps_with', 'recovered_pairs',
                    'wrong_edges', 'unmatched_edges', 'fstar_edges',
                    *PREDICTIONS}


def _no_verdict_in_edges(payload: dict) -> None:
    """Assert a G-E1 fail payload carries no match on a real map and no verdict.

    Exact key names, walked from the root; `spec_verbatim`, keyed P-E1 to
    P-E4 among the rest, is the registered wording and is not walked.
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


def _write_edges(payload: dict, out_dir: Path,
                 prefix: str) -> tuple[Path, str]:
    """S9's writer under this stage's name: a dated file, never an overwrite."""
    return dw._write_s9(payload, Path(out_dir), prefix)


def edges_meta(head: str, spec_rev: str, spec_sha256: str, porcelain: list,
               verbatim: dict, constants: dict, schema_constants: dict,
               sources: dict, v0_1: dict | None) -> dict:
    """The meta every file of this stage carries, the fail file's included."""
    return {'stage': 'edges_judge', 'head': head, 'spec_rev': spec_rev,
            'spec_sha256': spec_sha256,
            'registration': EDGES_REGISTRATION,
            'schema_registration': sj.SCHEMA_REGISTRATION,
            'tree_dirty': any(not ln.startswith('??') for ln in porcelain),
            'untracked': sum(1 for ln in porcelain if ln.startswith('??')),
            'spec': EDGES_SPEC_NAME, 'spec_verbatim': verbatim,
            'constants': constants, 'schema_constants': schema_constants,
            'truth_rho': TRUTH_RHO, 'decoy': DECOY, 'v0_1': v0_1,
            'sources': sources,
            'note': '§2-§4 come from edges_walls (no truth, D-E6), on schema '
                    'run 1\'s per-map files and the graph_walls files; truth '
                    'from schema_judge.truth on graph_walls.sdf_wall_faces, '
                    'cast through edges_walls.cast; M1 is '
                    'schema_judge.match_faces. Every map matched to truth goes '
                    'through EdgesDoor, which raises until G-E1 holds.'}


def _print_gate(gate: dict) -> None:
    names = {'2_truth': 'item 2', '3_decoy': 'item 3', '1_structure': 'item 1'}
    print('G-E1: ' + '  '.join(
        f'{names[n]} {"HOLD" if ok else ("FAIL" if ok is False else "-")}'
        for n, ok in ((n, gate['parts'][n]) for n in names))
        + f'   {"HOLD" if gate["all_ok"] else "FAIL"}')
    g2 = gate['2_truth']
    if g2 is not None and 'beside' in g2:
        b = g2['beside']
        print(f'  item 2: {g2["n_pairs"]} pairs, F* {g2["pairs_equal"]}, '
              f'widths in samples {g2["widths_equal"]}, '
              f'{g2["n_through_slab"]} through a slab; blocked '
              f'{b["blocked_total"]}, no hit {b["no_hit_total"]}, ties '
              f'{b["ties_total"]}; components {b["components"]}')
        for r in g2['pairs']:
            if not r['equal']:
                print(f'    {r["pair"]}: {r["samples"]} samples, expected '
                      f'{r["expected_samples"]}')
    g3 = gate['3_decoy']
    if g3 is not None and 'decoy_tally' in g3:
        t = g3['decoy_tally']
        print(f'  item 3: the decoy\'s {t["samples"]} samples, '
              f'{len(g3["decoy_facing_edges"])} facing edges, blocked '
              f'{t["blocked"]}; other edges unchanged '
              f'{g3["other_edges_unchanged"]}, tallies unchanged '
              f'{g3["tallies_unchanged"]}')
    g1 = gate['1_structure']
    if g1 is not None and 'maps' in g1:
        print(f'  item 1: corner and slab edges equal to the schema file on '
              f'{g1["n_maps_equal"]}/{g1["n_maps"]} maps')
        for m in g1['differing']:
            print(f'    {m["map"]}: corner {m["corner"]}, slab {m["slab"]}')


def _print_verdicts(verdicts: dict) -> None:
    for name in PREDICTIONS:
        v = verdicts[name]
        print(f'{name}: {"HOLD" if v["holds"] else "FAIL"}  ', end='')
        if 'n_holding' in v:
            print(f'{v["n_holding"]} of {v["of"]} maps, {v["rule"]}')
            for m in v['maps']:
                print(f'  {m}')
        else:
            print(f'{v["n_maps_with"]} of {v["of"]} maps with '
                  f'{v["what"]}, {v["rule"]}')
            for m in v['maps']:
                print(f'  {m}')


# ──────────────────────────── the stage, in order ────────────────────────────

def edges_stage(world: dict, spawn: list, constants: dict, meta: dict,
                inputs_of, out_dir: Path, run_dir: Path, builder_commit: str,
                t0: float | None = None) -> dict:
    """G-E1, then §5.3 on the 20 maps.

    The order, each step only after everything before it held (Q8):
    1. item 2, truth cast at rho 0.1 with the 12 faces and the 8 caps;
    2. item 3, the same with the decoy added;
    3. the inputs, loaded and checked (`inputs_of`), and nothing before this;
    4. the per-map files, written and read back, then item 1 on them;
    5. the door opens: M1 on the 20 maps, P-E1 to P-E4 on the five at
       cut1200, and cuts 60, 120 and 240 reported.

    On a failure the stage writes G-E1's results and stops at exit 2, after
    walking the payload for a verdict or a match. Reads no corpus file and
    resolves no spawn; `run_edges` is this function plus the I/O.
    """
    t0 = time.monotonic() if t0 is None else t0
    G2 = truth_graph(world, TRUTH_RHO)
    g2 = gate_2_truth(G2, constants)
    g3 = g1 = inputs = None
    records: dict = {}
    files: list = []
    if g2['ok']:
        G3 = truth_graph(world, TRUTH_RHO, decoy=DECOY)
        g3 = gate_3_decoy(G3, G2, DECOY)
    if g3 is not None and g3['ok']:
        inputs = inputs_of()
        records, files = write_per_map(inputs, run_dir, builder_commit)
        g1 = gate_1_structure(records, inputs['schema'], constants)

    gate = {'1_structure': g1, '2_truth': g2, '3_decoy': g3}
    parts = {n: (None if c is None else c['ok']) for n, c in gate.items()}
    gate['parts'] = parts
    gate['all_ok'] = all(v is True for v in parts.values())
    fstar = fstar_of(G2)
    door = EdgesDoor(records, {} if inputs is None else inputs['schema'],
                     spawn, G2['truth'],
                     None if inputs is None else inputs['checks']['tau_m'],
                     fstar)
    _print_gate(gate)

    if not door.open_after(g1, g2, g3):
        fail = dict(meta)
        fail.update({'stage': 'edges_judge_gate_fail',
                     'seconds': time.monotonic() - t0, 'G-E1': gate,
                     'per_map_files': files, 'n_door_calls': len(door.asked)})
        _no_verdict_in_edges(fail)
        path, _sha = _write_edges(fail, out_dir, 'edges_gate_fail')
        die('G-E1 failed; no map was matched to truth and no verdict was '
            f'computed, written or printed. {path}')

    # ── the door is open: M1 on the 20 maps ────────────────────────────────
    stems = inputs['stems']
    judged = {stem: door.judge(stem) for stem in stems}
    if door.asked != stems:
        die('one door call per map')
    rows = [row_of(stem, judged[stem]) for stem in stems]
    split = split_rows(rows, constants)
    verdicts = verdicts_from(rows, constants)
    _print_verdicts(verdicts)

    payload = dict(meta)
    payload.update({
        'seconds': time.monotonic() - t0,
        'rho_m': inputs['checks']['rho'], 'tau_m': inputs['checks']['tau_m'],
        'corpus': inputs['checks'], 'n_maps': len(stems),
        'truth': {'counts': G2['truth']['counts'], 'rho': TRUTH_RHO,
                  'facing': G2['facing'], 'below_w_min': G2['below_w_min'],
                  'tally': G2['tally'], 'components': G2['components'],
                  'through': G2['through'], 'occluders': G2['occluders']},
        'per_map_files': files,
        'G-E1': gate,
        'judgments': split['judged'],
        'verdicts': verdicts,
        'reported': {
            'cuts': split['reported'],
            'per_map': [dict(judged[stem], cut=r['cut']) for stem, r
                        in zip(stems, rows)]},
    })
    path, sha = _write_edges(payload, out_dir, 'edges_judge')
    out = dict(payload)
    out['written'] = {'path': str(path), 'sha256': sha}
    return out


def run_edges(out_dir: Path = OUT_DIR_DEFAULT,
              schema_results: Path = SCHEMA_RESULTS_DEFAULT,
              schema_maps: Path = SCHEMA_MAPS_DEFAULT,
              graph_dir: Path = gw.OUT_DIR_DEFAULT,
              splice_dir: Path = SPLICE_DIR_DEFAULT,
              variant: str = dw.CORPUS_VARIANT) -> dict:
    """The stage, end to end. Spec v0.2, §5.

    Before anything is loaded:
    1. REFUSES ON AN EDITED TREE.
    2. REGISTRATION BEFORE RESULT: `358650a` must be an ancestor of HEAD, and
       the spec at HEAD must carry "### 5.2 Gate G-E1" and "**P-E1." to
       "**P-E4.".
    3. §1-§5 AT HEAD MUST BE THE REGISTERED TEXT, byte for byte (Q15).
    4. Every number the stage applies is found in that text; the schema
       side's (tau_m, the spawn rev, the SDF prefix, the truth counts) in
       SPEC_b2_schema at HEAD, whose §1-§10 must be `5bf3ee2`'s (Q16).
    5. THREE FILES ARE PINNED BY HASH: schema run 1's results file, the F*
       file and the world SDF. Then the spawn rev; then truth without caps
       must reproduce the F* file (Q14, Q34).
    """
    t0 = time.monotonic()
    out_dir, splice_dir = Path(out_dir), Path(splice_dir)
    schema_results, schema_maps = Path(schema_results), Path(schema_maps)
    graph_dir = Path(graph_dir)

    porcelain = dw._git('status', '--porcelain').splitlines()
    dirty = [ln for ln in porcelain if not ln.startswith('??')]
    if dirty:
        die('the working tree has tracked changes, so a result file would name '
            'a commit it was not produced by:\n  ' + '\n  '.join(dirty))
    head = dw._git('rev-parse', 'HEAD')

    if not dw._is_ancestor(EDGES_REGISTRATION):
        die(f'{EDGES_REGISTRATION} registered §5, G-E1 and P-E1 to P-E4 and is '
            f'not an ancestor of HEAD {head[:7] or "(unknown)"}; the '
            'registration must be in history behind the code that tests it')
    spec_text = dw._git('show', f'HEAD:{SPEC_REL}')
    for needed in (GATE_HEADING, *(f'**{p}.' for p in PREDICTIONS)):
        if needed not in spec_text:
            die(f'the spec at HEAD carries no "{needed}"; this stage evaluates '
                'registered predictions behind a registered gate or it does '
                'not run')
    registered = dw._git('show', f'{EDGES_REGISTRATION}:{SPEC_REL}')
    if registered_body(spec_text) != registered_body(registered):
        die(f'§1-§5 at HEAD differ from {EDGES_REGISTRATION}; the stage '
            'applies the text that was registered, not an edited one')
    verbatim = edges_verbatim(spec_text)
    constants = edges_constants_from(verbatim)

    # The schema side, through the schema stage's own finders (Q16).
    if not dw._is_ancestor(sj.SCHEMA_REGISTRATION):
        die(f'{sj.SCHEMA_REGISTRATION} is not an ancestor of HEAD')
    schema_text = dw._git('show', f'HEAD:{sj.SPEC_REL}')
    schema_registered = dw._git('show', f'{sj.SCHEMA_REGISTRATION}:'
                                        f'{sj.SPEC_REL}')
    if sj.registered_body(schema_text) != sj.registered_body(schema_registered):
        die(f'SPEC_b2_schema §1-§10 at HEAD differ from '
            f'{sj.SCHEMA_REGISTRATION}; M1 and its tolerance are that text')
    schema_constants = sj.schema_constants_from(sj.schema_verbatim(schema_text))
    spec_rev = dw._git('log', '-1', '--format=%h', '--', SPEC_REL)
    builder_commit = dw._git('log', '-1', '--format=%h', '--', EW_REL)

    sources = {}
    for name, path, want, what in (
            ('schema_results', schema_results, SCHEMA_RESULTS_SHA256,
             "schema run 1's results give each per-map file's sha256 (Q6)"),
            ('fstar_out', splice_dir / FSTAR_OUT, FSTAR_SHA256,
             '§5.1 pins F* to it (Q14)')):
        if not path.is_file():
            die(f'{path} is missing; {what}')
        sha = dw._sha256(path)
        if sha != want:
            die(f'{path.name} has sha256 {sha}, not the {want} the stage is '
                'stated against; that is a different file')
        sources[name] = {'file': str(path), 'sha256': sha}
    sources['spec'] = {'file': SPEC_REL,
                       'sha256': dw._sha256(REPO_ROOT / SPEC_REL)}
    sources['schema_spec'] = {'file': sj.SPEC_REL,
                              'sha256': dw._sha256(REPO_ROOT / sj.SPEC_REL)}
    sdf_sha = dw._sha256(REPO_ROOT / sj.WORLD_SDF_REL)
    if not sdf_sha.startswith(schema_constants['files']['sdf_sha256_prefix']):
        die(f'{sj.WORLD_SDF_REL} has sha256 {sdf_sha}, and SPEC_b2_schema §1 '
            f'names {schema_constants["files"]["sdf_sha256_prefix"]}…')
    sources['world_sdf'] = {'file': sj.WORLD_SDF_REL, 'sha256': sdf_sha}
    parsed = parse_fstar_out((splice_dir / FSTAR_OUT).read_text(
        encoding='utf-8'))

    import fit_world_transform as fwt                         # noqa: PLC0415

    spawn = fwt.resolve_spawn_poses(sj.check_spawn_rev(schema_constants))

    world = gw.sdf_wall_faces(gw.L_MIN)
    G01 = truth_graph(world, TRUTH_RHO, caps=False)
    sj.check_truth(G01['truth'], schema_constants)
    check_edges_truth(G01, constants)
    check_fstar_out(parsed, G01, constants, sdf_sha)

    stems = dw.corpus_stems(variant)
    pattern = schema_constants['rows']['pattern']
    want_stems = [pattern.replace('{K}', str(k)).replace('{C}', str(c))
                  for k in range(schema_constants['rows']['k'][0],
                                 schema_constants['rows']['k'][1] + 1)
                  for c in schema_constants['rows']['cuts']]
    if sorted(stems) != sorted(want_stems) \
            or len(stems) != constants['gate_maps']['n']:
        die(f'the schema rows are {len(want_stems)} stems as {pattern} and the '
            f'corpus is {len(stems)} stems of variant {variant}')

    def inputs_of() -> dict:
        t = time.monotonic()
        inputs = load_inputs(schema_results, schema_maps, graph_dir, stems)
        inputs['checks'] = corpus_checks(inputs, constants, schema_constants)
        print(f'  inputs: {len(stems)} maps, {time.monotonic() - t:.1f} s',
              flush=True)
        return inputs

    meta = edges_meta(head, spec_rev, sources['spec']['sha256'], porcelain,
                      verbatim, constants, schema_constants, sources,
                      v0_1_summary(G01))
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    return edges_stage(world, spawn, constants, meta, inputs_of, out_dir,
                       out_dir / f'edges_maps_{stamp}', builder_commit, t0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out-dir', type=Path, default=OUT_DIR_DEFAULT,
                    help='where the results file and the per-map directory go')
    ap.add_argument('--schema-results', type=Path,
                    default=SCHEMA_RESULTS_DEFAULT,
                    help="schema run 1's results file, pinned by sha256")
    ap.add_argument('--schema-maps', type=Path, default=SCHEMA_MAPS_DEFAULT,
                    help="schema run 1's 20 per-map files")
    ap.add_argument('--graph-dir', type=Path, default=gw.OUT_DIR_DEFAULT,
                    help='the 20 graph_walls files, <stem>.json')
    ap.add_argument('--splice-dir', type=Path, default=SPLICE_DIR_DEFAULT,
                    help='where the pinned F* file is')
    args = ap.parse_args()
    run_edges(args.out_dir, args.schema_results, args.schema_maps,
              args.graph_dir, args.splice_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
