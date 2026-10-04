"""The stem -> bag mapping in experiments/analysis/diagnose_karto.py.

One question only: which recording does a stem's ray set come from.
SPEC_b2_divergence_walls §4 requires the read to go through
graph_walls.bag_variant_of, because the travel gate lives in the BAG --
minimum_travel_distance and minimum_travel_heading are 0.0 in the b2maps replay
configs, so Karto gates nothing -- while an overlay build replays the recording
of the run it is named after. Reading the ungated bag for a gated map would
silently admit scans that map never saw.

NOTHING HERE OPENS A BAG. bag_path_for builds a name and returns a Path; the
cases below include a cut that was never recorded, which is the cheapest proof
that the function does no I/O. The rest of diagnose_karto needs bags, a world
SDF and git, and is not tested here.

Importing diagnose_karto costs numpy, PyYAML, graph_walls and trinary_map, all
of which CI's ros:jazzy container has. It is loaded by location, the way
test_robot_divergence.py:47 loads its script.
"""

import importlib.util
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))
ANALYSIS = os.path.join(REPO_ROOT, 'experiments', 'analysis')


def load_module(name):
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(ANALYSIS, f'{name}.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


dk = load_module('diagnose_karto')


def test_old_corpus_stem_reads_the_bag_it_always_read():
    """A plain stem must resolve to exactly the path hardcoded before the fix.

    This is the regression half of the change: the 40-map record in
    diagnose_karto.json was built from these paths, and a variant-aware read
    that moved them would invalidate it.
    """
    p = dk.bag_path_for('b2maps_k0_cut1200_robot0', 0, 1200)
    assert p.name == 'b2maps_k0_cut1200_slamin'
    assert p.parent == dk.gw.BAGS_DIR


def test_gated_extfix_stem_reads_the_gated_bag():
    """The gate belongs in the bag name; the binary variant must not."""
    p = dk.bag_path_for('b2maps_k1_cut60_gated_extfix_robot1', 1, 60)
    assert p.name == 'b2maps_k1_cut60_gated_slamin'
    assert 'extfix' not in p.name


def test_old_corpus_stem_reads_the_run_artefacts_it_always_read():
    """The run name must not move for a plain stem either.

    The replay config and the map->odom capture are addressed by RUN, not by the
    bag name, and the 40-map record was built with the plain one.
    """
    assert dk.run_name_for('b2maps_k0_cut1200_robot0', 0, 1200) \
        == 'b2maps_k0_cut1200'
    assert dk.gw.run_variant_of('b2maps_k0_cut1200_robot0') == ''


def test_variant_stem_reads_its_own_run_artefacts():
    """Every variant token belongs in a RUN name, _extfix included.

    The opposite of the bag, which carries the gate only: an _extfix replay
    rendered its own config and its own map->odom capture, and those describe
    it, while the recording it replayed was the plain run's.
    """
    stem = 'b2maps_k1_cut60_gated_extfix_robot1'
    assert dk.run_name_for(stem, 1, 60) == 'b2maps_k1_cut60_gated_extfix'
    assert dk.bag_path_for(stem, 1, 60).name == 'b2maps_k1_cut60_gated_slamin'


def test_the_mapping_opens_nothing():
    """A cut nobody recorded still returns a path, so no bag was touched."""
    p = dk.bag_path_for('b2maps_k3_cut7777_gated_robot3', 3, 7777)
    assert p.name == 'b2maps_k3_cut7777_gated_slamin'
    assert not p.exists()
