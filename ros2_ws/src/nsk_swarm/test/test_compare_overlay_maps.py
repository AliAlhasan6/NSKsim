"""R1, R2 and R3, and the three breaks, on synthetic maps written to tmp_path.

The real runs are stages C to E and they take hours, so the checks are exercised
here on grids small enough to count by hand. That is the point of the file: each
check is shown to HOLD on an input it should accept and to FAIL on one it should
reject, and each named break is shown to fail the checks it is pre-registered to
fail. A break that has only ever been argued about is not a break.

Written through predict_gated_maps.write_pgm, so the fixtures are PGM/YAML pairs in
the same convention as every other map here and the tool reads them through
trinary_map exactly as it will read the real ones.

Also pins the stem grammar: b2maps_k{K}_cut{C}[_gated][_ovl|_extfix]_robot{K},
including that `_ovl` and `_extfix` are mutually exclusive and that a bag path
never carries either.
"""

import importlib.util
import os
import sys

import numpy as np
import pytest

# test/ -> nsk_swarm -> src -> ros2_ws -> repo root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
ANALYSIS = os.path.join(REPO_ROOT, 'experiments', 'analysis')
SLAM = os.path.join(REPO_ROOT, 'experiments', 'slam')
for p in (ANALYSIS, SLAM):
    if p not in sys.path:
        sys.path.insert(0, p)


def _load(name):
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(ANALYSIS, f'{name}.py'))
    mod = importlib.util.module_from_spec(spec)
    sys.modules.setdefault(name, mod)
    spec.loader.exec_module(mod)
    return mod


gw = _load('graph_walls')
pg = _load('predict_gated_maps')
cm = _load('compare_overlay_maps')

OCC, FREE, UNK = gw.OCC, gw.FREE, gw.UNKNOWN
RHO = 0.1
ORIGIN = (-1.25, 2.75)


def write(d, stem, grid, origin=ORIGIN, rho=RHO):
    """A PGM/YAML pair from a BOTTOM-UP grid, the way every map here is written."""
    pg.write_pgm(d / stem, np.asarray(grid, dtype=np.uint8), rho, origin)
    return stem


def base_grid():
    """A 6 x 5 bottom-up grid: a wall up the right-hand column, free inside."""
    g = np.full((5, 6), FREE, dtype=np.uint8)
    g[:, 5] = OCC
    g[0, :] = UNK
    return g


def grown(g, far_col=OCC, far_row=UNK):
    """`g` with one column and one row added at the far (+x, +y) side."""
    h, w = g.shape
    out = np.full((h + 1, w + 1), far_row, dtype=np.uint8)
    out[:h, :w] = g
    out[:h, w] = far_col
    return out


# ─────────────────────────── the stem grammar ────────────────────────────────

@pytest.mark.parametrize('stem,variants,run,bag', [
    ('b2maps_k0_cut60_robot0', (), '', ''),
    ('b2maps_k0_cut60_gated_robot0', ('gated',), '_gated', '_gated'),
    ('b2maps_k0_cut60_gated_ovl_robot0', ('gated', 'ovl'), '_gated_ovl', '_gated'),
    ('b2maps_k2_cut1200_gated_extfix_robot2', ('gated', 'extfix'),
     '_gated_extfix', '_gated'),
    ('b2maps_k3_cut240_ovl_robot3', ('ovl',), '_ovl', ''),
])
def test_the_stem_grammar_separates_the_run_variant_from_the_bag_variant(
        stem, variants, run, bag):
    """A binary variant names the BINARY, so it must never reach a bag path.

    The bag an _ovl run replays is the one the run it is named after replayed --
    the overlay changed the compiler's output, not the recording. A bag path
    carrying _ovl would name a file nobody ever made.
    """
    assert gw.stem_variants(stem) == variants
    assert gw.run_variant_of(stem) == run
    assert gw.bag_variant_of(stem) == bag


@pytest.mark.parametrize('bad', [
    'b2maps_k0_cut60_gated_gated_robot0',     # a token twice
    'b2maps_k0_cut60_ovl_extfix_robot0',      # patched and not patched at once
    'b2maps_k0_cut60_extfix_gated_robot0',    # out of order
    'b2maps_k0_cut60_robot0_ovl',             # token after the robot
])
def test_malformed_stems_are_refused_rather_than_half_parsed(bad):
    assert gw.stem_variants(bad) == ()
    with pytest.raises(SystemExit) as exc:
        gw._k_and_cut(bad, 'the test needs them')
    assert exc.value.code == 2


def test_a_well_formed_stem_with_mismatched_robot_is_refused_on_that_rule():
    """b2maps_k0_cut60_ovl_robot1 parses -- and is still refused.

    The grammar accepts it, so stem_variants reads its tokens; the refusal comes
    from the separate rule that these runs have one explorer each. Keeping the two
    apart matters: a grammar failure and a wrong-robot failure want different
    fixes.
    """
    stem = 'b2maps_k0_cut60_ovl_robot1'
    assert gw.stem_variants(stem) == ('ovl',)
    with pytest.raises(SystemExit) as exc:
        gw._k_and_cut(stem, 'the test needs them')
    assert exc.value.code == 2


def test_an_overlay_bag_variant_is_refused_by_name_not_by_a_missing_file():
    """diagnose_extent.variant_bag must say WHY, not die on a path that is absent.

    'bag not found' would send a reader looking for a recording that was never
    made; naming the mistake sends them to bag_variant_of.
    """
    de = _load('diagnose_extent')
    with pytest.raises(SystemExit) as exc:
        de.variant_bag(0, 60, '_gated_ovl')
    assert exc.value.code == 2


def test_strip_binary_variant_keeps_the_gate_and_drops_only_the_binary():
    assert cm.strip_binary_variant('b2maps_k0_cut60_gated_ovl_robot0') == \
        'b2maps_k0_cut60_gated_robot0'
    assert cm.strip_binary_variant('b2maps_k0_cut60_gated_extfix_robot0') == \
        'b2maps_k0_cut60_gated_robot0'
    assert cm.plain_stem('b2maps_k0_cut60_gated_extfix_robot0') == \
        'b2maps_k0_cut60_robot0'
    assert cm.other_robot_stem('b2maps_k0_cut60_gated_ovl_robot0') == \
        'b2maps_k1_cut60_gated_ovl_robot0'.replace('_robot0', '_robot1')


# ─────────────────────────────────── R1 ──────────────────────────────────────

def test_r1_holds_on_identical_bytes_and_fails_on_one_changed_cell(tmp_path):
    g = base_grid()
    write(tmp_path, 'b2maps_k0_cut60_gated_robot0', g)
    write(tmp_path, 'b2maps_k0_cut60_gated_ovl_robot0', g)
    r = cm.r1('b2maps_k0_cut60_gated_ovl_robot0', tmp_path, None)
    assert r['holds'] and r['bytes_identical'] and r['geometry_identical']
    assert r['sha256'] == r['sha256_reference']

    # One cell different is enough. R1 is a byte test precisely so that it is.
    g2 = g.copy()
    g2[2, 2] = OCC
    write(tmp_path, 'b2maps_k0_cut60_gated_ovl_robot0', g2)
    bad = cm.r1('b2maps_k0_cut60_gated_ovl_robot0', tmp_path, None)
    assert not bad['holds'] and not bad['bytes_identical']


def test_r1_fails_when_the_origin_moved_even_with_identical_pixels(tmp_path):
    """Same pixels at a different origin is a different map of the world."""
    g = base_grid()
    write(tmp_path, 'b2maps_k0_cut60_gated_robot0', g)
    write(tmp_path, 'b2maps_k0_cut60_gated_ovl_robot0', g,
          origin=(ORIGIN[0] + RHO, ORIGIN[1]))
    r = cm.r1('b2maps_k0_cut60_gated_ovl_robot0', tmp_path, None)
    assert r['bytes_identical'], 'the PGM bytes are the same'
    assert not r['geometry_identical'] and not r['holds']


def test_r1_skips_rather_than_passes_when_the_reference_is_absent(tmp_path):
    write(tmp_path, 'b2maps_k0_cut60_gated_ovl_robot0', base_grid())
    r = cm.r1('b2maps_k0_cut60_gated_ovl_robot0', tmp_path, None)
    assert 'status' in r and 'holds' not in r


# ─────────────────────────────────── R2 ──────────────────────────────────────

def test_r2_holds_when_only_the_far_row_and_column_were_added(tmp_path):
    g = base_grid()
    write(tmp_path, 'b2maps_k0_cut60_gated_ovl_robot0', g)
    write(tmp_path, 'b2maps_k0_cut60_gated_extfix_robot0', grown(g))
    r = cm.r2('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, None)
    assert r['holds']
    assert r['grew_by_exactly_one_per_axis'] and r['origin_unchanged']
    assert r['n_overlap_differing'] == 0
    assert r['reference'] == 'b2maps_k0_cut60_gated_ovl_robot0', \
        "the overlay's own unpatched build is the closer control"
    assert 'overlay' in r['reference_kind']


def test_r2_fails_on_one_changed_cell_inside_the_old_extent(tmp_path):
    g = base_grid()
    write(tmp_path, 'b2maps_k0_cut60_gated_ovl_robot0', g)
    big = grown(g)
    big[2, 2] = OCC                      # inside the OLD extent: not allowed
    write(tmp_path, 'b2maps_k0_cut60_gated_extfix_robot0', big)
    r = cm.r2('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, None)
    assert not r['holds'] and r['n_overlap_differing'] == 1


def test_r2_fails_when_the_grid_grew_by_two(tmp_path):
    g = base_grid()
    write(tmp_path, 'b2maps_k0_cut60_gated_ovl_robot0', g)
    write(tmp_path, 'b2maps_k0_cut60_gated_extfix_robot0', grown(grown(g)))
    r = cm.r2('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, None)
    assert not r['holds'] and not r['grew_by_exactly_one_per_axis']


def test_r2_falls_back_to_the_deb_map_and_says_so_when_no_ovl_build_exists(
        tmp_path):
    """Then "identical to the unpatched map" leans on R1, and the row says it."""
    g = base_grid()
    write(tmp_path, 'b2maps_k0_cut60_gated_robot0', g)
    write(tmp_path, 'b2maps_k0_cut60_gated_extfix_robot0', grown(g))
    r = cm.r2('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, None)
    assert r['holds']
    assert r['reference'] == 'b2maps_k0_cut60_gated_robot0'
    assert 'only as good as R1' in r['reference_kind']


def test_r2_fails_when_the_origin_moved(tmp_path):
    """The patch leaves rOffset alone, so a moved origin is not this patch."""
    g = base_grid()
    write(tmp_path, 'b2maps_k0_cut60_gated_ovl_robot0', g)
    write(tmp_path, 'b2maps_k0_cut60_gated_extfix_robot0', grown(g),
          origin=(ORIGIN[0] - RHO / 2, ORIGIN[1]))
    r = cm.r2('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, None)
    assert not r['origin_unchanged'] and not r['holds']


# ─────────────────────────────────── R3 ──────────────────────────────────────

def _r3_fixture(tmp_path, pred_far_col=OCC, pred_far_row=UNK):
    g = base_grid()
    write(tmp_path, 'b2maps_k0_cut60_gated_ovl_robot0', g)
    write(tmp_path, 'b2maps_k0_cut60_gated_extfix_robot0', grown(g))
    pred = tmp_path / 'pred'
    pred.mkdir(exist_ok=True)
    write(pred, 'b2maps_k0_cut60_robot0',
          grown(g, far_col=pred_far_col, far_row=pred_far_row))
    return pred


def test_r3_holds_when_the_added_cells_match_the_prediction(tmp_path):
    pred = _r3_fixture(tmp_path)
    r = cm.r3('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, pred, None)
    assert r['holds'] and r['agreement'] == pytest.approx(1.0)
    assert r['n_observed'] > 0, 'an all-unknown comparison would be vacuous'


def test_r3_fails_when_the_prediction_put_free_where_the_map_has_wall(tmp_path):
    pred = _r3_fixture(tmp_path, pred_far_col=FREE)
    r = cm.r3('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, pred, None)
    assert not r['holds'] and r['agreement'] < cm.R3_MIN


def test_r3_refuses_to_resample_two_grids_of_different_size(tmp_path):
    """Lining them up by resampling would absorb the error R3 is looking for."""
    pred = _r3_fixture(tmp_path)
    write(pred, 'b2maps_k0_cut60_robot0', grown(grown(base_grid())))
    r = cm.r3('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, pred, None)
    assert 'status' in r and r['holds'] is False


# ────────────────────────────── the three breaks ─────────────────────────────

def test_break_other_robot_fails_r1_and_r2_and_cannot_speak_for_r3(tmp_path):
    g = base_grid()
    for k in (0, 1):
        # robot_1's maps differ from robot_0's, which is what the break relies on.
        gk = g if k == 0 else np.flipud(g)
        write(tmp_path, f'b2maps_k{k}_cut60_gated_robot{k}', gk)
        write(tmp_path, f'b2maps_k{k}_cut60_gated_ovl_robot{k}', gk)
        write(tmp_path, f'b2maps_k{k}_cut60_gated_extfix_robot{k}', grown(gk))
    pred = tmp_path / 'pred'
    pred.mkdir(exist_ok=True)
    for k in (0, 1):
        gk = g if k == 0 else np.flipud(g)
        write(pred, f'b2maps_k{k}_cut60_robot{k}', grown(gk))

    assert cm.r1('b2maps_k0_cut60_gated_ovl_robot0', tmp_path, None)['holds']
    assert cm.r2('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, None)['holds']
    assert cm.r3('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, pred,
                 None)['holds']

    b1 = cm.r1('b2maps_k0_cut60_gated_ovl_robot0', tmp_path, 'other-robot')
    b2 = cm.r2('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, 'other-robot')
    assert not b1['holds'] and not b2['holds']
    assert b1['reference'].endswith('robot1')
    # R3 is deliberately NOT registered against this break: it reads only the
    # added row and column, and on real data two robots' grids differ in size, so
    # R3 refuses the pair and is SKIPPED. interior-row is R3's break.
    assert 'R3' not in cm.BREAK_MUST_FAIL['other-robot']


def test_break_shift_one_cell_fails_r1_and_r2(tmp_path):
    g = base_grid()
    write(tmp_path, 'b2maps_k0_cut60_gated_robot0', g)
    write(tmp_path, 'b2maps_k0_cut60_gated_ovl_robot0', g)
    write(tmp_path, 'b2maps_k0_cut60_gated_extfix_robot0', grown(g))
    assert cm.r1('b2maps_k0_cut60_gated_ovl_robot0', tmp_path, None)['holds']
    assert cm.r2('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, None)['holds']

    b1 = cm.r1('b2maps_k0_cut60_gated_ovl_robot0', tmp_path, 'shift-one-cell')
    b2 = cm.r2('b2maps_k0_cut60_gated_extfix_robot0', tmp_path,
               'shift-one-cell')
    assert not b1['holds'], 'a whole-cell slip must not read as identical'
    assert not b2['holds']


def test_break_interior_row_fails_r3(tmp_path):
    """The interior must not agree with the prediction's added row and column.

    If it did, R3 would be satisfied by cells the patch did not add, which is the
    one way R3 could look like it was working while measuring nothing.
    """
    pred = _r3_fixture(tmp_path)
    assert cm.r3('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, pred,
                 None)['holds']
    b = cm.r3('b2maps_k0_cut60_gated_extfix_robot0', tmp_path, pred,
              'interior-row')
    assert not b['holds']


def test_every_break_is_registered_against_the_checks_it_must_fail():
    """The table the tool prints and the table these tests use are one table."""
    assert set(cm.BREAKS) == set(cm.BREAK_MUST_FAIL)
    for b, must in cm.BREAK_MUST_FAIL.items():
        assert must, f'{b} must name at least one check'
        assert set(must) <= set(cm.CHECKS)


# ──────────────────────── the overlay hook in the shell ──────────────────────

HOOK_SCRIPT = os.path.join(SLAM, 'run_offline_maps.sh')


def _hook_block():
    """The SLAM_TOOLBOX_OVERLAY block, lifted out of the real script.

    Extracted rather than copied so the test cannot drift away from what the
    script actually does.
    """
    text = open(HOOK_SCRIPT).read()
    start = text.index('if [[ -n "${SLAM_TOOLBOX_OVERLAY:-}" ]]; then')
    end = text.index('\nfi\n', start) + len('\nfi\n')
    return text[start:end]


def _run_hook(tmp_path, env, preamble='set -uo pipefail\n',
              tail='\necho "REACHED-END"\n'):
    """Run the lifted block under `bash`, with the runner's own options.

    preamble is the state the hook is entered in -- the real script's
    `set -uo pipefail` by default -- and tail is what runs after it, which is
    how the option-restoring tests probe the shell the hook left behind.
    """
    import subprocess
    script = tmp_path / 'hook.sh'
    script.write_text(preamble
                      + 'die() { echo "DIED: $*"; exit 1; }\n'
                      + _hook_block() + tail)
    return subprocess.run(['bash', str(script)], capture_output=True, text=True,
                          env={**os.environ, **env})


# colcon's generated setup.bash reads COLCON_TRACE with no default (line 11 of
# install/setup.bash: `if [ -n "$COLCON_TRACE" ]`), which under the runner's
# set -u is fatal. Every fixture below sources a script that does the same thing,
# because one that only exported would pass against a hook that cannot source a
# real overlay at all -- which is what shipped, and what stopped stage C.
COLCON_LIKE_SETUP = ('if [ -n "$COLCON_TRACE" ]; then echo "# trace"; fi\n'
                     'export NSK_HOOK_PROOF=sourced\n')


def _overlay(tmp_path, body=COLCON_LIKE_SETUP):
    ovl = tmp_path / 'install'
    ovl.mkdir()
    (ovl / 'setup.bash').write_text(body)
    return ovl


# Reads an unbound variable, so the shell dies under nounset and survives
# without it. The probe for which state the hook left behind.
NOUNSET_PROBE = ('\necho "PROBE: [${NSK_NO_SUCH_VARIABLE}]"\n'
                 'echo "PAST-PROBE"\n')


def test_the_overlay_hook_is_a_no_op_when_the_variable_is_unset(tmp_path):
    """The whole claim behind "default unset is byte-identical"."""
    env = dict(os.environ)
    env.pop('SLAM_TOOLBOX_OVERLAY', None)
    import subprocess
    script = tmp_path / 'hook.sh'
    script.write_text('set -uo pipefail\n'
                      'die() { echo "DIED: $*"; exit 1; }\n'
                      + _hook_block() + '\necho "REACHED-END"\n')
    r = subprocess.run(['bash', str(script)], capture_output=True, text=True,
                       env=env)
    assert r.returncode == 0
    assert r.stdout.strip() == 'REACHED-END', \
        'an unset overlay must print nothing and source nothing'


def test_the_overlay_hook_refuses_a_path_with_no_setup_bash(tmp_path):
    r = _run_hook(tmp_path, {'SLAM_TOOLBOX_OVERLAY': str(tmp_path / 'nope')})
    assert r.returncode == 1 and 'DIED' in r.stdout
    assert 'has no setup.bash' in r.stdout


def test_the_overlay_hook_sources_a_valid_overlay_and_says_so(tmp_path):
    """The overlay is a REAL colcon workspace, so the fixture reads COLCON_TRACE.

    This is the test that stage C's failure bought: with a setup.bash that only
    exported, this passed while the hook was unable to source any overlay colcon
    had generated. Under set -u the unbound expansion is fatal and bash exits 1
    at the source line -- 'COLCON_TRACE: unbound variable', rc 1, nothing
    written, which is exactly what 2026-10-03 produced for k0, k2 and the break.
    """
    ovl = _overlay(tmp_path)
    r = _run_hook(tmp_path, {'SLAM_TOOLBOX_OVERLAY': str(ovl)})
    assert r.returncode == 0, \
        f'the hook could not source a colcon overlay: {r.stderr.strip()}'
    assert 'sourced' in r.stdout and 'REACHED-END' in r.stdout


def test_the_overlay_hook_puts_nounset_back_on_after_sourcing(tmp_path):
    """nounset is off for the source and ONLY for the source.

    The rest of the script relies on set -u -- an unset DUR_N or a typo'd knob
    has to be a failure, not an empty string -- so a hook that left nounset off
    would quietly weaken every line after it.
    """
    ovl = _overlay(tmp_path)
    r = _run_hook(tmp_path, {'SLAM_TOOLBOX_OVERLAY': str(ovl)},
                  tail=NOUNSET_PROBE)
    assert 'sourced' in r.stdout, 'the hook did not get as far as sourcing'
    assert 'PAST-PROBE' not in r.stdout and r.returncode != 0, \
        'nounset was not restored: the probe read an unbound variable and lived'
    assert 'unbound variable' in r.stderr


def test_the_overlay_hook_does_not_turn_nounset_on_if_it_was_off(tmp_path):
    """Restored EXACTLY, which is not the same as restored to set -u.

    The block is lifted into tests and could be lifted into another runner; it
    reports the shell it was handed, so `set +o nounset` on entry must still be
    `set +o nounset` on exit.
    """
    ovl = _overlay(tmp_path)
    r = _run_hook(tmp_path, {'SLAM_TOOLBOX_OVERLAY': str(ovl)},
                  preamble='set +u\nset -o pipefail\n', tail=NOUNSET_PROBE)
    assert r.returncode == 0, r.stderr
    assert 'PROBE: []' in r.stdout and 'PAST-PROBE' in r.stdout, \
        'the hook turned nounset on in a shell that had it off'
