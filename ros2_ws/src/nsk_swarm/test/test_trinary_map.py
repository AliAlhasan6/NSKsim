"""The shared reader in experiments/analysis/trinary_map.py.

Every reader of this project's saved maps goes through this module, so the rule it
states is pinned here rather than in each caller:

    the thresholds split occupied from free, the reserved byte 205 OVERRIDES
    them, and a fourth pixel value is a refusal rather than a guess.

The test that matters most is the dull-looking one: an all-205 grid classified
under the very thresholds its own YAML carries must come back UNKNOWN, not FREE.
That is the defect this module exists for -- shade(205) = 50/255 = 0.196 is below
free_thresh 0.25, so a classifier that consults only the thresholds reports every
unknown cell as free, which once reported all 20 offline b2maps grids as fully
known and turned Jaccard into extent overlap.

Two corpus variants are pinned, because both are real: 183 YAMLs carry
free_thresh 0.25 (which does NOT recover 205) and three -- rung1b/1c/1d_robot0 --
carry 0.196 (which DOES, by 7.8e-5, since shade(205) = 0.196078...). classify()
must return the same three states either way, and check_thresholds() must report
the difference without treating either as an error.

The module is loaded by location the way test_robot_divergence.py loads
robot_divergence.py: experiments/ is not an importable package. Importing it costs
numpy and PyYAML only -- no PIL, no scipy, no ROS -- so this file must keep
collecting in CI's ros:jazzy container, where a module-level scipy import takes
the whole nsk_swarm suite down.
"""

import glob
import importlib.util
import os

import numpy as np
import pytest
import yaml

# test/ -> nsk_swarm -> src -> ros2_ws -> repo root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
SCRIPT = os.path.join(REPO_ROOT, 'experiments', 'analysis', 'trinary_map.py')

# Overridable, and the override exists for one reason: the corpus guards at the
# bottom of this file must SKIP rather than FAIL where there is no corpus, and CI
# is exactly that case (experiments/maps is gitignored, so a fresh clone has
# none). A skipif is evaluated at COLLECTION time, so an in-test monkeypatch
# cannot reach it -- pointing this at an empty directory is the only way to
# exercise the absent-corpus path without moving the real maps aside.
#
#     NSK_TRINARY_MAPS_DIR=/nonexistent python3 -m pytest test/test_trinary_map.py
#
# A guard that has never been observed to skip is a guard nobody has tested.
MAPS_DIR = os.environ.get('NSK_TRINARY_MAPS_DIR',
                          os.path.join(REPO_ROOT, 'experiments', 'maps'))


def load_module():
    spec = importlib.util.spec_from_file_location('trinary_map', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


tm = load_module()

OCC, FREE, UNK = tm.OCC, tm.FREE, tm.UNKNOWN

# The two free_thresh values that exist in experiments/maps/. 0.25 is what all
# three writers emit; 0.196 is the hand-made rung1 era.
FT_STOCK = 0.25
FT_RECOVERING = 0.196


def write_pgm(path, rows, maxval=255, header_extra=''):
    """A P5 PGM from a list of byte-lists, for the header-walk tests."""
    px = np.array(rows, dtype=np.uint8)
    h, w = px.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{header_extra}{w} {h}\n{maxval}\n'.encode())
        f.write(px.tobytes())
    return px


# ───────────────────────────────── read_pgm ──────────────────────────────────

def test_read_pgm_reads_a_plain_p5(tmp_path):
    want = write_pgm(tmp_path / 'm.pgm', [[0, 254, 205], [205, 0, 254]])
    got = tm.read_pgm(tmp_path / 'm.pgm')
    assert got.tolist() == want.tolist()
    assert got.dtype == np.uint8
    assert got.shape == (2, 3)


def test_read_pgm_accepts_both_str_and_path(tmp_path):
    """pgm_extent.py's callers pass a str; pgm_agreement.py's pass a Path.

    test_run_health.py:1890 is the str caller, and it is not edited by this
    change -- so the shared reader has to take both or that suite breaks.
    """
    write_pgm(tmp_path / 'm.pgm', [[0, 254], [205, 0]])
    as_path = tm.read_pgm(tmp_path / 'm.pgm')
    as_str = tm.read_pgm(str(tmp_path / 'm.pgm'))
    assert as_str.tolist() == as_path.tolist()


def test_read_pgm_skips_comments_and_odd_whitespace(tmp_path):
    """'#' comment lines and extra whitespace between header fields.

    nav2's own writer emits neither, but the header walk claims to handle them and
    a claim in a docstring is not a test.
    """
    want = write_pgm(tmp_path / 'm.pgm', [[0, 254, 205]],
                     header_extra='# written by something else\n  \t')
    assert tm.read_pgm(tmp_path / 'm.pgm').tolist() == want.tolist()


def test_read_pgm_refuses_a_non_p5_magic(tmp_path):
    p = tmp_path / 'm.pgm'
    p.write_bytes(b'P2\n2 1\n255\n0 254\n')      # plain ASCII PGM, not binary
    with pytest.raises(SystemExit) as exc:
        tm.read_pgm(p)
    assert exc.value.code != 0


def test_read_pgm_refuses_a_truncated_pixel_block(tmp_path):
    """A header promising more pixels than follow.

    The failure this catches is a half-written map from a killed saver, which
    would otherwise reshape-error somewhere far from the cause.
    """
    p = tmp_path / 'm.pgm'
    p.write_bytes(b'P5\n4 4\n255\n' + bytes([254] * 5))
    with pytest.raises(SystemExit) as exc:
        tm.read_pgm(p)
    assert exc.value.code != 0


# ─────────────────────────────────── load ────────────────────────────────────

def write_yaml(path, image, res=0.05, origin=(-1.0, 2.0, 0.0), negate=0,
               occupied_thresh=0.65, free_thresh=FT_STOCK):
    path.write_text(
        f'image: {image}\nmode: trinary\nresolution: {res}\n'
        f'origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n'
        f'negate: {negate}\noccupied_thresh: {occupied_thresh}\n'
        f'free_thresh: {free_thresh}\n')


def test_load_returns_geometry_and_counts_by_reserved_byte(tmp_path):
    write_pgm(tmp_path / 'm.pgm', [[0, 254, 254], [205, 205, 0]])
    write_yaml(tmp_path / 'm.yaml', 'm.pgm', res=0.05, origin=(-1.0, 2.0, 0.0))
    m = tm.load(tmp_path / 'm.pgm')
    assert (m['width'], m['height']) == (3, 2)
    assert m['resolution'] == 0.05
    assert m['origin'] == [-1.0, 2.0, 0.0]
    # Counts are by reserved byte, i.e. what is in the file -- not by threshold.
    assert (m['occupied'], m['free'], m['unknown'], m['other']) == (2, 2, 2, 0)


def test_load_counts_a_fourth_byte_as_other(tmp_path):
    """'other' is what check_thresholds refuses on; load only reports it."""
    write_pgm(tmp_path / 'm.pgm', [[0, 254, 128]])
    write_yaml(tmp_path / 'm.yaml', 'm.pgm')
    m = tm.load(tmp_path / 'm.pgm')
    assert m['other'] == 1


def test_load_refuses_a_missing_sidecar(tmp_path):
    write_pgm(tmp_path / 'm.pgm', [[0, 254]])
    with pytest.raises(SystemExit):
        tm.load(tmp_path / 'm.pgm')


# ─────────────────────────────────── shade ───────────────────────────────────

def test_shade_of_the_unknown_byte_is_50_over_255():
    assert tm.shade(205.0, 0) == pytest.approx(50 / 255)


def test_shade_honours_negate():
    """negate flips which end of the range means occupied.

    Under negate=1 the unknown byte reads 0.804, which is ABOVE occupied_thresh
    0.65 -- 205 would be a WALL. That is why check_thresholds dies on such a YAML
    rather than classifying under it.
    """
    assert tm.shade(205.0, 1) == pytest.approx(205 / 255)
    assert tm.shade(205.0, 1) > 0.65
    assert tm.shade(0.0, 0) == pytest.approx(1.0)
    assert tm.shade(0.0, 1) == pytest.approx(0.0)


# ────────────────────────────────── classify ─────────────────────────────────

def test_classify_recovers_the_three_reserved_bytes():
    px = np.array([[0, 254, 205]], dtype=np.uint8)
    got = tm.classify(px, negate=0, occupied_thresh=0.65, free_thresh=FT_STOCK)
    assert got.tolist() == [[OCC, FREE, UNK]]


def test_classify_does_not_let_the_thresholds_call_unknown_free():
    """THE defect this module exists for.

    shade(205) = 50/255 = 0.196, which is below free_thresh 0.25 -- so a
    classifier consulting only the thresholds returns FREE for every unknown cell.
    Applied literally it reported all 20 offline b2maps grids as fully known:
    Jaccard became extent overlap and the coverage measurement silently measured
    nothing.
    """
    assert tm.shade(205.0, 0) < FT_STOCK        # the trap, in one line
    px = np.full((4, 5), 205, dtype=np.uint8)
    got = tm.classify(px, negate=0, occupied_thresh=0.65, free_thresh=FT_STOCK)
    assert np.all(got == UNK)
    assert not np.any(got == FREE)


def test_classify_puts_the_threshold_band_in_unknown():
    """A byte inside [free_thresh, occupied_thresh] is unknown on its own merits.

    Not via the reserved-byte override -- these bytes are not 205. The band the
    stock thresholds map to unknown is 90..191, and no pixel of any corpus map
    occupies it, but the arithmetic has to be right anyway.
    """
    px = np.array([[90, 140, 191]], dtype=np.uint8)
    got = tm.classify(px, negate=0, occupied_thresh=0.65, free_thresh=FT_STOCK)
    assert got.tolist() == [[UNK, UNK, UNK]]


def test_classify_honours_negate():
    px = np.array([[0, 254]], dtype=np.uint8)
    got = tm.classify(px, negate=1, occupied_thresh=0.65, free_thresh=FT_STOCK)
    assert got.tolist() == [[FREE, OCC]]


def test_classify_is_identical_under_both_corpus_free_thresholds():
    """0.25 and 0.196 must give the same three states.

    The reserved byte overrides under 0.25; the thresholds themselves already say
    unknown under 0.196. Different routes, same answer -- which is why neither
    value needs a special case anywhere downstream.
    """
    px = np.array([[0, 254, 205, 90, 191]], dtype=np.uint8)
    stock = tm.classify(px, 0, 0.65, FT_STOCK)
    recovering = tm.classify(px, 0, 0.65, FT_RECOVERING)
    assert stock.tolist() == recovering.tolist()


# ─────────────────────────────  check_thresholds ─────────────────────────────

def test_check_thresholds_reports_that_205_is_not_recovered_at_0_25():
    info = tm.check_thresholds(0, 0.65, FT_STOCK, other=0, where='t')
    assert info['unknown_byte_recovered_by_thresholds'] is False
    assert info['unknown_byte_band_under_thresholds'] == 'free'
    assert info['threshold_unknown_band_bytes'] == [90, 191]


def test_check_thresholds_reports_that_205_is_recovered_at_0_196():
    """The rung1b/1c/1d_robot0 case, and how thin it is.

    0.196 works only because shade(205) = 0.196078... is above it, by 7.8e-5.
    Pinned so that the margin is on the record: 0.1961 would NOT recover it, and
    a release step that rewrites thresholds should pick a value with room (0.1
    leaves ~0.096 either side) rather than copy this one.
    """
    info = tm.check_thresholds(0, 0.65, FT_RECOVERING, other=0, where='t')
    assert info['unknown_byte_recovered_by_thresholds'] is True
    assert info['unknown_byte_band_under_thresholds'] == 'unknown'
    # The margin, stated as arithmetic rather than as a comment.
    assert tm.shade(205.0, 0) - FT_RECOVERING == pytest.approx(7.843e-5, rel=1e-3)
    assert tm.shade(205.0, 0) < 0.1961      # how little room there is


def test_check_thresholds_is_not_an_error_either_way():
    """Recovery is REPORTED, not required. Both corpus variants must pass."""
    for ft in (FT_STOCK, FT_RECOVERING):
        info = tm.check_thresholds(0, 0.65, ft, other=0, where='t')
        assert info['unknown_byte'] == UNK


def test_check_thresholds_dies_on_negate_1():
    """A negate: 1 YAML describes some other image, so this refuses to classify.

    Under negate=1 the reserved occupied byte 0 reads as FREE and 205 reads as a
    WALL (shade 0.804 > 0.65). Silently classifying under it would invert a map.
    """
    with pytest.raises(SystemExit) as exc:
        tm.check_thresholds(1, 0.65, FT_STOCK, other=0, where='t')
    assert exc.value.code == 2


def test_check_thresholds_dies_on_a_fourth_pixel_value():
    with pytest.raises(SystemExit) as exc:
        tm.check_thresholds(0, 0.65, FT_STOCK, other=7, where='t')
    assert exc.value.code == 2


def test_check_thresholds_dies_on_unordered_thresholds():
    with pytest.raises(SystemExit) as exc:
        tm.check_thresholds(0, FT_STOCK, 0.65, other=0, where='t')
    assert exc.value.code == 2


def test_check_thresholds_dies_on_thresholds_outside_0_1():
    with pytest.raises(SystemExit) as exc:
        tm.check_thresholds(0, 1.4, FT_STOCK, other=0, where='t')
    assert exc.value.code == 2


# ────────────────────────── where a cell sits ────────────────────────────────
# CELL_CENTRE_OFFSET is the second rule this module owns. It is 0.0 because
# slam_toolbox publishes Karto's grid offset verbatim as the map origin and Karto
# centres cell i on that offset -- so cell i's centre is origin + i*res. Six files
# used to restate this, two of them differently.


def test_the_cell_centre_offset_is_kartos_and_the_citations_are_in_the_module():
    """The value, and that the reason for it travels with it.

    A bare 0.0 is indistinguishable from a bug, so the two source citations are
    required to be present: a later reader who finds the constant has to be able
    to check it against slam_toolbox without being told where to look.
    """
    assert tm.CELL_CENTRE_OFFSET == 0.0
    src = open(SCRIPT).read()
    assert 'visualization_utils.hpp:108-129' in src
    assert 'Karto.h:4421-4436' in src


def test_cell_index_inverts_cell_centre_for_every_cell():
    """The round trip, on a non-round origin so no coincidence can carry it."""
    origin, res = -3.2749999, 0.1
    i = np.arange(-50, 50)
    assert np.array_equal(tm.cell_index(tm.cell_centre(origin, i, res),
                                        origin, res), i)


def test_a_cell_spans_half_a_cell_either_side_of_its_centre():
    """cell_low_edge is the centre minus half a cell, and the span is contiguous.

    Which is the whole difference from the ROS reading: under CELL_CENTRE_OFFSET
    0.0 cell i STARTS half a cell below origin + i*res rather than at it.
    """
    origin, res = 0.0, 0.1
    assert tm.cell_low_edge(origin, 3, res) == pytest.approx(0.25)
    assert tm.cell_low_edge(origin, 4, res) == pytest.approx(0.35)
    assert tm.cell_centre(origin, 3, res) == pytest.approx(0.30)
    # Just inside each edge indexes to the cell; just outside, to its neighbours.
    assert tm.cell_index(0.2501, origin, res) == 3
    assert tm.cell_index(0.3499, origin, res) == 3
    assert tm.cell_index(0.2499, origin, res) == 2
    assert tm.cell_index(0.3501, origin, res) == 4


def test_sample_is_invariant_to_the_offset_and_a_common_shift():
    """Why no sample()-based measurement moved when the convention was fixed.

    sample() asks for cell_centre(x0, i, res) and answers with
    cell_index(x, origin, res). Both carry the offset, so changing it moves the
    query and the lookup together and cancels; and shifting x0 and origin by the
    same amount cancels too -- which is why compare_gated_maps' hand -rho/2 on
    BOTH origins was inert, and why removing it changed nothing.

    The offset matters only where a WORLD coordinate meets a cell index, which is
    cell_index() called on something that is not a lattice point of this grid.
    """
    px = (np.arange(12, dtype=np.uint8) * 20).reshape(3, 4)
    m = {'px': px, 'width': 4, 'height': 3, 'origin': [0.37, -1.23]}
    res = 0.1
    want = tm.sample(m, res, 0.37, -1.23, 4, 3)

    saved = tm.CELL_CENTRE_OFFSET
    try:
        for off in (0.0, 0.5, 0.25):
            tm.CELL_CENTRE_OFFSET = off
            m_off = dict(m)
            assert np.array_equal(tm.sample(m_off, res, 0.37, -1.23, 4, 3), want)
            shifted = dict(m, origin=[0.37 - res / 2.0, -1.23 - res / 2.0])
            assert np.array_equal(
                tm.sample(shifted, res, 0.37 - res / 2.0, -1.23 - res / 2.0,
                          4, 3), want)
    finally:
        tm.CELL_CENTRE_OFFSET = saved


def test_the_provenance_dict_carries_the_value_and_both_citations():
    """Every output JSON embeds this, so it has to say enough to be read alone."""
    p = tm.cell_centre_provenance()
    assert p['cell_centre_offset_cells'] == tm.CELL_CENTRE_OFFSET
    assert 'visualization_utils.hpp:108-129' in p['why']
    assert 'Karto.h:4421-4436' in p['why']
    assert 'trinary_map.py' in p['defined_in']


# ─────────────────────────────────── sample ──────────────────────────────────

def test_sample_places_a_map_where_the_origin_says():
    """Convention A: PGM row 0 is MAXIMUM y, so sample flips the rows once.

    The occupied cell is written in PGM ROW 0 (the top), and must therefore come
    back in the TOP row of the bottom-up lattice, i.e. the last row of the array.
    Getting this backwards mirrors every map about the x axis, which looks
    plausible in a viewer and is wrong everywhere.
    """
    px = np.array([[OCC, UNK], [UNK, UNK]], dtype=np.uint8)
    m = {'px': px, 'width': 2, 'height': 2, 'origin': [0.0, 0.0]}
    got = tm.sample(m, res=1.0, x0=0.0, y0=0.0, w=2, h=2)
    # Bottom-up: row index 1 is the higher y, which is PGM row 0.
    assert got[1].tolist() == [OCC, UNK]
    assert got[0].tolist() == [UNK, UNK]


def test_sample_fills_unknown_outside_the_map():
    px = np.array([[FREE]], dtype=np.uint8)
    m = {'px': px, 'width': 1, 'height': 1, 'origin': [0.0, 0.0]}
    got = tm.sample(m, res=1.0, x0=-1.0, y0=-1.0, w=3, h=3)
    assert got[1, 1] == FREE
    assert int(np.count_nonzero(got == UNK)) == 8


def test_sample_is_a_three_state_resampler_over_classify():
    """Handed a classified grid, sample moves states around and invents none."""
    raw = np.array([[0, 254], [205, 0]], dtype=np.uint8)
    cls = tm.classify(raw, 0, 0.65, FT_STOCK)
    m = {'px': cls, 'width': 2, 'height': 2, 'origin': [0.0, 0.0]}
    got = tm.sample(m, res=1.0, x0=0.0, y0=0.0, w=2, h=2)
    assert set(np.unique(got).tolist()) <= {OCC, FREE, UNK}


# ───────────────────────────── the corpus invariant ──────────────────────────
# pgm_extent.py deliberately classifies by reserved byte rather than through
# classify(), which is only safe while every saved map is strictly trinary and
# every sidecar carries the fields a threshold reading would need. That is an
# assumption about the files on disk, so it is checked against them.

CORPUS = sorted(glob.glob(os.path.join(MAPS_DIR, '**', '*.pgm'), recursive=True))

# Same shape as test_robot_divergence.py:122 -- a module-level skipif naming the
# absent path, so CI reports these as skipped rather than red. experiments/maps is
# gitignored, so "absent" is the normal case everywhere except a working tree that
# has built maps.
needs_corpus = pytest.mark.skipif(
    not CORPUS, reason=f'no *.pgm under {MAPS_DIR} (gitignored; absent in CI)')


@needs_corpus
def test_every_saved_map_is_strictly_trinary():
    offenders = []
    for path in CORPUS:
        px = tm.read_pgm(path)
        bad = int(np.count_nonzero((px != OCC) & (px != FREE) & (px != UNK)))
        if bad:
            offenders.append((os.path.basename(path), bad))
    assert offenders == [], (
        'these maps carry a byte the reserved-byte rule says nothing about, so '
        'pgm_extent.py must stop counting bytes and route through classify(): '
        f'{offenders}')


@needs_corpus
def test_every_sidecar_carries_the_fields_a_threshold_read_would_need():
    missing = []
    for path in CORPUS:
        meta_path = os.path.splitext(path)[0] + '.yaml'
        if not os.path.isfile(meta_path):
            missing.append((os.path.basename(meta_path), 'absent'))
            continue
        meta = yaml.safe_load(open(meta_path).read())
        for key in ('resolution', 'origin', 'negate', 'occupied_thresh',
                    'free_thresh'):
            if key not in meta:
                missing.append((os.path.basename(meta_path), key))
    assert missing == [], f'sidecars missing fields: {missing}'


@needs_corpus
def test_the_corpus_uses_exactly_the_two_known_free_thresholds():
    """0.25 on what the writers emit, 0.196 on the hand-made rung1 maps.

    A third value appearing means somebody changed a writer or hand-edited a
    sidecar, and the claim in this module's docstring about which maps recover 205
    would need rechecking.
    """
    seen = {}
    for path in CORPUS:
        meta_path = os.path.splitext(path)[0] + '.yaml'
        if not os.path.isfile(meta_path):
            continue
        ft = float(yaml.safe_load(open(meta_path).read())['free_thresh'])
        seen.setdefault(ft, []).append(os.path.basename(meta_path))
    assert set(seen) <= {FT_STOCK, FT_RECOVERING}, (
        f'unexpected free_thresh values: '
        f'{ {k: v[:3] for k, v in seen.items() if k not in (FT_STOCK, FT_RECOVERING)} }')
    # And every one of them classifies 205 as unknown, by one route or the other.
    px = np.full((2, 2), 205, dtype=np.uint8)
    for ft in seen:
        assert np.all(tm.classify(px, 0, 0.65, ft) == UNK)
