# B2 freeze record

Written on 19 Oct 2026 by Claude Code from Ali's rulings, and reviewed in chat. Drafts
1 and 2 are pinned in `experiments/logs/splice/`: `B2_FREEZE_draft1.md` (`051e34e9…`)
and `B2_FREEZE_draft2.md` (`f692585c…`).

**Input.** `experiments/logs/splice/b2_freeze_inventory.md` (INV), sha256
`1162c336da81bf4edb62b4da4a30c91f1d91f09fc1aa3dc03b74e8510109d307` (block (19 Oct)).
It was also checked by reading its content against this session's own report of it,
and the two match.

**Sourcing.**
- Every statement cites its source: `file:line`, or a result file with its sha256.
- Numbers are copied from their sources, never recomputed.
- **INFERRED** marks anything concluded rather than read.
- Ali's decisions of 19 Oct 2026 are cited as **ruling (19 Oct)**. They come from the
  prompts for drafts 1 and 2, and this file is their repo record.
- Ali's terminal output of 19 Oct 2026, pasted for draft 2, is cited as **block
  (19 Oct)**. Its values are recorded in this file: the inventory hash (header), TP's
  last commit (§1) and the S4 hash (§2, row 48).

**Step 0 (this session).**
- `git rev-parse HEAD` → `8de8dc541b2f762fb337d5f0c932eabb09e32bd4`
- `git status --short` → `?? .vscode/`. Block (19 Oct) shows the same.

**Short names.** Citations are `name:line`.

| Short | File |
|---|---|
| W | `docs/specs/SPEC_b2_wall_predicate.md` |
| D | `docs/specs/SPEC_b2_divergence_walls.md` |
| S | `docs/specs/SPEC_b2_schema.md` |
| E | `docs/specs/SPEC_b2_edges.md` |
| G | `docs/specs/SPEC_b2_steady_gate.md` |
| TP | `docs/specs/SPEC_truth_pose_replay.md` |
| SJ | `experiments/analysis/schema_judge.py` |
| EJ | `experiments/analysis/edges_judge.py` |
| DW | `experiments/analysis/divergence_walls.py` |
| TSJ | `ros2_ws/src/nsk_swarm/test/test_schema_judge.py` |
| TEJ | `ros2_ws/src/nsk_swarm/test/test_edges_judge.py` |
| H0914 | `docs/handoffs/HANDOFF_2026-09-14_b2_unblocked_designed_run_owed.md` |
| H1004 | `docs/handoffs/HANDOFF_2026-10-04_b2_wall_graphs_and_final_map_corpus.md` |
| H1006 | `docs/handoffs/HANDOFF_2026-10-06_b2_divergence_on_walls.md` |
| H1015 | `docs/handoffs/HANDOFF_2026-10-15_b2_parked_shadows_o4_closed.md` |
| H1017 | `docs/handoffs/HANDOFF_2026-10-17_b2_schema_v0_2_registered.md` |
| H18 | `docs/handoffs/HANDOFF_2026-10-18_b2_schema_run1.md` |
| H19 | `docs/handoffs/HANDOFF_2026-10-19_b2_face_graph_judged.md` |
| RL | `.git/logs/HEAD` |
| plan | `experiments/logs/splice/edges_build_plan.md`, sha256 `2015120f…` (H19 §5, line 147) |
| inv | `experiments/logs/splice/edge_spec_inventory.md`, sha256 `d187ece2…` (H19 §5, line 143) |
| INV | `experiments/logs/splice/b2_freeze_inventory.md` |

---

## 0. Purpose

**B2 is frozen on 19 Oct 2026 at `8de8dc5`** (ruling (19 Oct); HEAD as in Step 0).

**The rule: no spec text is edited, for any of the six specs in §1** (ruling (19 Oct)).
So the errata live here (§3), not in the specs. Any later change is a new spec version,
logged below with a dated line.

**What pins the text differs by spec.** Each finding below cites where it was read.

- **S: schema_judge compares §1–§10 byte for byte.**
  - SJ:78–81: "Ruling 4 (Ali, 17 Oct 2026): §1-§10 at HEAD must be byte-identical to the
    registered text. Ancestry alone does not catch a later edit." The range runs from
    `'## 1. The world'` to `'## 11. Open, before the stage'`.
  - SJ:5–10 adds the other refusals. Before anything loads, the stage "refuses an edited
    tree, requires the registration to be an ancestor of HEAD, stores the registered
    text verbatim and finds in that text every number it applies".
  - SJ:75 holds the registration, `SCHEMA_REGISTRATION = '5bf3ee2'`.
  - Outside the byte range:
    - SJ:104 and SJ:117–119 read §0's heading and the labels of D-S5 to D-S7, and
      TSJ:160–164 assert their text.
    - S's title, header and §11 fall outside the range. Whether other code reads them
      was not checked.
  - Tests:
    - TSJ:102 and TSJ:123–125 read S from the worktree.
    - TSJ:27 lists Y6, "§1-§10 at HEAD edited (ruling 4)", expected "refuses".
    - TEJ:100–101 and TEJ:134–136 read S as well.
  - H1017:175–176: "**The title keeps "(v0.2, draft)"** like the divergence spec. The
    stage stores the text verbatim, so it is not to be edited."
- **E: edges_judge compares §1–§5 and leaves §6 out.**
  - EJ:80–83: "Q15: §1-§5 at HEAD must be byte-identical to the registered text. §6
    (Open items) is left out, so closing an open item does not block a rerun."
  - EJ:4–8 makes the same refusals as SJ.
  - EJ:77 holds the registration, `EDGES_REGISTRATION = '358650a'`.
  - Tests: TEJ:99 and TEJ:129–131 read E from the worktree.
- **D: S13 reads the divergence spec at HEAD.**
  - DW:6990–6995: S13 "REFUSES ON AN EDITED TREE"; `2a88223` "must be an ancestor of
    HEAD, and the spec at HEAD must carry "P18. REGISTERED", "P19. REGISTERED" and
    "G11 (gate)."". Also: "The registered text is read from the spec AT HEAD by script
    and stored verbatim, and every number S13 applies is found in it."
  - The code is at DW:7008–7019. DW:7012 reads the text with `_git('show',
    f'HEAD:{SPEC_REL}')`.
  - No byte comparison against the registered commit appears in DW:7008–7019.
    `s13_verbatim`, which S13 calls at DW:7018, was not read.
  - S10, S11 and S12 each require their own registration commit to be an ancestor of
    HEAD (D:920–921, D:1099–1100, D:1342–1344).
  - Tests: `test_divergence_walls.py` was not read for spec reads. That it pins D's
    text is INFERRED.
- **W, G and TP: no pinning file was found (INFERRED).**
  - `graph_walls.py` and its tests were not read in full.
  - `steady_gate.py` was not read. H0914:30–32 says it was "committed citing sections"
    of G.
  - No stage or test that was read refers to TP.

### Change log

- 19 Oct 2026: frozen at `8de8dc5`. No spec changed.

---

## 1. Frozen specs

Sources for the columns:
- **sha256 at HEAD** is `sha256sum docs/specs/*.md`, run in this session against a
  clean tree.
- **Last commit** comes from `git log`, run by Ali:
  - the five B2 specs, confirmed in ruling (19 Oct);
  - TP, from block (19 Oct): `git log -1 --format='%h %ad %s' --date=short --
    docs/specs/SPEC_truth_pose_replay.md` printed `03d666c 2026-09-19 docs/handoffs:
    the 09-18 and 09-19 known-pose handoffs`.
- `sha256sum docs/specs/*.md` listed exactly these six files when it was run, before
  this record was added to `docs/specs/`. So INV §0's limit 1 is closed for `.md` files
  in `docs/specs/`.

| Spec | Title version | Last commit | sha256 at HEAD | Pinned by |
|---|---|---|---|---|
| W | "v2" (W:1) | `cdba182` | `ef93367e7f121800a8234f42121861b411866c0a0355605ef3c92da6ab90afae` | None found (INFERRED; see §0) |
| D | "(v0.22, draft)" (D:1) | `0c4aff7` | `790893f2cec2343eaf930c4dc36554b930d322eb14df95b153a6a6f8f6bcc03e` | DW:6990–6995 and DW:7008–7019 (S13 reads the spec at HEAD). S10–S12 require their registration commits as ancestors (D:920–921, D:1099–1100, D:1342–1344). `test_divergence_walls.py`: INFERRED. |
| S | "(v0.2, draft)" (S:1) | `5bf3ee2` | `8dd6da2ddfac39093b0669772e4465b257b8de90a51f37d0e3a38479a1b4eef6` | SJ:78–81 (§1–§10 byte for byte); SJ:71, SJ:75, SJ:104–119; TSJ:102, TSJ:123–125, TSJ:150–164; TEJ:100–101, TEJ:134–136. EJ:32–33 borrows `schema_judge.schema_verbatim` and its constants. |
| E | "(v0.2, 18 Oct 2026)" (E:1) | `358650a` | `9497a3798adc61e60e8c81be21cd5ef7b8b206a1e8c37b329411473772e4e940` | EJ:80–83 (§1–§5 byte for byte, §6 left out); EJ:73, EJ:77; TEJ:99, TEJ:129–131 |
| G | none found (G:1) | `6724221` | `7c820d7a407539a17abcbbd30cf0fca9b212e4dd428902acd2da551605142da1` | None found (INFERRED; see §0) |
| TP (input spec) | none. TP:1 is "SPEC — truth-pose replay (known-pose mapping probe)"; status lines at TP:8–12, dated 2026-09-18 and 2026-09-19 | `03d666c` (2026-09-19), "docs/handoffs: the 09-18 and 09-19 known-pose handoffs" (block (19 Oct)) | `312cbb4fc609c3fa9838f3cbe73751077dc61bdc4e9bdb28d1487d216667d78f` | None found (INFERRED; see §0) |

**TP's last commit differs from draft 1.**
- Draft 1 inferred `00e76e5` from RL:101, "docs/specs: SPEC_truth_pose_replay.md, the
  2x2 results", the last reflog subject that names TP.
- `git log` gives `03d666c`. That is RL:104, whose subject names only handoffs.
- So a commit touched TP after `00e76e5` without naming it in its subject, and the
  inference from reflog subjects was wrong for TP.

The hashes agree with earlier pins:
- S's sha256 equals the value H1017:37–38 expected at `5bf3ee2`.
- E's equals the registered copy's (`spec_edges_v0_2_register.md`:74–77).

No earlier pinned sha256 of W, D or G was found in the sources read.

TP is listed as an input spec by ruling (19 Oct). TP:427–429 also records the order in
which the two files landed: "`SPEC_b2_steady_gate.md` was owed to `docs/specs/`
(handoff §7) and landed in the preceding commit".

---

## 2. Verdict register

**Counting.**
- One row per labelled gate, prediction or check in INV §5.
- The rows include those that the specs do not hold: S3/S4, G2, G-S1, P-S1 to P-S8,
  G-E1, P-E1 to P-E4, V1 to V5 and S0 to S3.
- A row whose item was judged on more than one corpus is counted by its last recorded
  verdict. The earlier verdict is quoted beside it.
- F1–F3 restate B1, B2 and A8–A10 on the final corpus, but Stage F registered them
  separately (W:1304–1305), so they keep their own rows.

**Hashes.**
- The S4 file's sha256 was taken at the freeze and is labelled "hashed at freeze,
  19 Oct 2026".
- Every other hash that no source records is marked "not recorded".

### W, wall predicate (rows 1–39)

| # | Item | Verdict | Verdict source | Result file, sha256 |
|---|---|---|---|---|
| 1 | A8 | HELD 20/20, final corpus (F3) | W:1311 | `_validation_20261004T021555Z.json`, `555ef95f2cc6608c40c925fa1b56e717a80f731f93fc6a06122c841bffc6babe` (SJ:124–126; S:283–284). That this is the F3 run is INFERRED. |
| 2 | A9 | HELD 5/5 robots, final corpus (F3) | W:1311 | as row 1 (INFERRED) |
| 3 | A10 | HELD 20/20, final corpus (F3) | W:1311 | as row 1 (INFERRED) |
| 4 | B1 | FAILED 4/20 as registered, plain corpus (W:382). POST HOC in-grid HOLDS 20/20 (W:632–633). Final corpus: "the original B1 holds 20/20" (H1004:161) | W:382, W:1309 | per-map `<map>.json` `checks` (W:602–604); sha256 not recorded |
| 5 | B2 | FAILED 1/20, plain corpus (W:382–383; 1.352 % at W:637–638). Final corpus HELD, worst 0.249 % (F2) | W:1310 | per-map files; sha256 not recorded |
| 6 | B3 | Held 20/20, +0 (W:376). Final corpus 19/19, 1 of 20 not judged | W:1329 | per-map files (INFERRED); sha256 not recorded |
| 7–13 | A1–A7 | No verdict recorded. W:206: "Every check is shown failing with its named break before it is trusted." | W:210–220 | the test suite; no result file |
| 14 | B4 | No verdict recorded | W:242 | the test suite; no result file |
| 15 | Dwell P1 | FAILED | W:400 | `diagnose_dwell.json` (W:400); sha256 not recorded |
| 16 | Dwell P2 | FAILED | W:400 | as row 15 |
| 17 | Dwell P3 | HELD, "P3 held 20/20" (H1004:149) | W:400 | as row 15 |
| 18 | Grid-phase G1 | failed | H1004:153 | `diagnose_phase.json` (W:400); sha256 not recorded |
| 19 | Grid-phase G2 | failed | H1004:153 | as row 18 |
| 20 | K3, segment count within 1 % on 20/20 | "**FAILS on 1 of 20**" | W:545–546 | file not named; sha256 not recorded |
| 21 | K3, A8 median falls on 20/20 | "**FAILED: it falls on 15/20 and rises on 5/20**" | W:555–557, W:641 | file not named; sha256 not recorded |
| 22 | X1 (W §12.4) | HELD | W:748 | `diagnose_extent.json` (W:675); sha256 not recorded |
| 23 | X2 (W §12.4) | HELD | W:754 | as row 22 |
| 24–26 | T1, T2, T3 | None recorded (INV §5). VOID by ruling (19 Oct); see §4.5 | W:786–788 | none |
| 27 | E1 | HELD 20/20 | W:853 | `b2maps_extent/predicted/_summary.json` (W:887); sha256 not recorded |
| 28 | E2 | HELD 20/20 | W:854 | `diagnose_extent_fix.json` (W:886); sha256 not recorded |
| 29 | E3, predicted maps | HELD 5/5 | W:855 | as row 28 |
| 30 | E3, patched maps | HELD 5/5, worst 1.06 pp | W:1259 | `diagnose_extent.json` (INFERRED from W:1246–1252); sha256 not recorded |
| 31 | R0 | 2/2 at Stage C (W:1019) and 2/2 at Stage D (W:1070) | W:1019, W:1070 | file not named; sha256 not recorded |
| 32 | R1 | 3/3 HELD | W:1073; H1004:85 | file not named; sha256 not recorded |
| 33 | R2 | 2/2 HELD (W:1073), then 20/20 HELD | W:1242; H1004:86 | file not named; sha256 not recorded |
| 34 | R3 | FAILED 1/2 (W:1077), then 19/20; "the 99.5 % threshold is unmoved" | W:1242–1244; H1004:87 | file not named; sha256 not recorded |
| 35 | C1 | 20/20, under a heading "reported only" (W:493) | W:495 | `experiments/logs/b2maps_gated/comparison.json` (W:497); sha256 not recorded |
| 36 | C2 | 5/5 at 100 %. Its break "failed C2 on 5/5 cut1200 maps as pre-registered" | W:495–497 | as row 35 |
| 37 | F1 | HELD | W:1309 | registration in `stage_f_preregistration.txt` (W:1305); results file not named; sha256 not recorded |
| 38 | F2 | HELD, worst 0.249 % | W:1310 | as row 37 |
| 39 | F3 | HELD | W:1311 | as row 37. The results file is INFERRED to be row 1's |

### D, divergence (rows 40–75)

| # | Item | Verdict | Verdict source | Result file, sha256 |
|---|---|---|---|---|
| 40 | G0 | HOLD 20/20 | H1006:83 | `s3_gates_20261004T201944Z.json` (H1006:79), `dd0b3796…` (H1015:103; only this prefix is recorded) |
| 41 | G1 | HOLD 80/80 | H1006:85 | as row 40 |
| 42 | X1 (frame sign) | failed as intended, "0/80 hold" | H1006:86 | as row 40 |
| 43 | X6 | failed as intended, "132 cells" | H1006:84 | as row 40 |
| 44 | C0 | FAILED, "0/20 under v0.1, 4/20 under v0.3" | D:327 | file not named; sha256 not recorded |
| 45 | P1 | VOID | D:326–328 | none |
| 46 | P2 | VOID | D:329–331 | none |
| 47 | P5 | VOID | D:340–343 | none |
| 48 | P3 | HOLD 20/20 | H1006:92 | `s4_predictions_20261005T230035Z.json` (H1006:88), sha256 `4f173f4cde62d958dc1adb987dc8b7ad87393d7725546209278eee632ab991e0`, hashed at freeze, 19 Oct 2026 (block (19 Oct)) |
| 49 | P4 | HOLD 80/80 | H1006:94 | as row 48 |
| 50 | P6 | HOLD 16/16 | H1006:93 | as row 48 |
| 51 | G2 | HOLD 20/20 | H1015:212 | commit `25ba70e` (RL:185); file not named; sha256 not recorded |
| 52 | G3 | HOLD 20/20 | D:400 | `s5_viewpoint_20261006T212758Z.json`, `f99e001d517734ea4b8d737436f0e9243122c0edd9d30f65a241dde39e89bb67` (D:394–395) |
| 53 | P7 | HOLD 19/20 | D:404 | as row 52 |
| 54 | P8 | HOLD 20/20 | D:409 | as row 52 |
| 55 | G4 | HOLD 820/820 | D:501 | `s6_cutmatrix_20261008T225604Z.json`, `a3d584f310e509c33bcca09c2fe641d4914dd4f768146f10c1ebf340f58b2f13` (D:496–497) |
| 56 | P9 | FAIL 40/60 | D:507 | as row 55 |
| 57 | G5 | HOLD | D:589 | `s7_gainloss_20261009T000606Z.json`, `b8b7cb6c4dc57e8df48ea9baa975dc2e85b8eddbeed7e75c71fe52a271461c21` (D:584–585) |
| 58 | P10 | HOLD 20/20 | D:593 | as row 57 |
| 59 | G6 | HOLD | D:707 | `s8_cellstates_20261009T010602Z.json`, `f71f4597d3e050417d65d092295b9ccf1ec0337bbf54af39868418da9c6c31c4` (D:701–702) |
| 60 | P11 | FAIL 21/213 | D:711 | as row 59 |
| 61 | G7 | HOLD | D:802 | `s9_segmentation_20261009T013501Z.json`, `42c7505992ff8047d069cb7092dcdeaf3e8644193ff36484f3892425d307deab` (D:796–797) |
| 62 | P12 | HOLD 75/78 | D:812 | as row 61 |
| 63 | G8 | HOLD | D:970 | `s10_segments_20261011T134922Z.json`, `fa750aa97fc08c5a1f177b78019832c3c5794ef42a8898d9d8f6d1d7127407e2` (D:964–965) |
| 64 | P13a | FAIL 2/5 | D:971 | as row 63 |
| 65 | P13b | HOLD 12/14 | D:973 | as row 63 |
| 66 | P14 | HOLD 0/22 | D:975 | as row 63 |
| 67 | G9 | HOLD | D:1153 | `s11_trace_20261012T233219Z.json`, `006a7f8fc2774cf1324b613d42e70d8e4d103bd8e8298624a8630522c58663dd` (D:1147–1148) |
| 68 | P15 | FAIL 1/22 | D:1154 | as row 67 |
| 69 | P16 | HOLD 0/22 | D:1155 | as row 67 |
| 70 | G10 | HOLD | D:1360 | `s12_mirror_20261013T101707Z.json`, `247faaa851e9c29cbd5fdd29f4b03e8f27a9b3faef2968ed721be9405486f1ba` (D:1354–1355) |
| 71 | P17 | HOLD 60/60 | D:1370 | as row 70 |
| 72 | G11 | HOLD | D:1615 | `s13_shadows_20261014T095408Z.json`, `cd1ada604646b786157a87b3a4adf1f1701aaf2ff613de6c9eb2822081a08023` (H1015:41–42; D:1611–1612) |
| 73 | X7 (decoy) | no hit return in 20 of 20 maps, as intended | D:1621 | as row 72 |
| 74 | P18 | HOLD 20/20; "the largest Δ is 0.0023 (rounded)" | D:1624–1625 | as row 72 |
| 75 | P19 | HOLD 20/20; "Δ is 0 in all 20 pairs" | D:1627–1628 | as row 72 |

### S, schema (rows 76–84)

All rows share one results file:
`experiments/logs/schema/judge_5a72769_20261018T103953Z/schema_judge_20261018T104734Z.json`,
sha256 `84ffdb5a5d6a75ee9e50bb27658df1878353ea0227dfe6eac98ecc3ae5379f18`
(H18:26–27; EJ:111–112). None of these verdicts is in S (INV §3, S4).

| # | Item | Verdict | Verdict source |
|---|---|---|---|
| 76 | G-S1, checks 1–3 (check 3 is decoy X-S1) | HOLD | H18:58–62 |
| 77 | P-S1 | 20 of 20 inside corners found; bound ≥ 18 | H18:66 |
| 78 | P-S2 | 0 maps with a wrong inside corner; bound ≤ 1 | H18:67 |
| 79 | P-S3 | 20 of 20 slabs found; bound ≥ 18 | H18:68 |
| 80 | P-S4 | 0 maps with a wrong pair or slab end; bound ≤ 1 | H18:69 |
| 81 | P-S5 | 40 of 40 slab ends found; bound ≥ 36 | H18:70 |
| 82 | P-S6 | 0 of 5 cut1200 maps with an opening; bound ≤ 1 | H18:71 |
| 83 | P-S7 | 0 maps with an unmatched-part inside corner; bound ≤ 1 | H18:72 |
| 84 | P-S8 | 0 maps with an unmatched-part pair or slab end; bound ≤ 1 | H18:73 |

H18:12–13 sums it up: "Gate G-S1 held, and all eight registered predictions hold."

### E, edges (rows 85–89)

All rows share one results file:
`experiments/logs/edges/judge_e2ae698_20261019T084358Z/edges_judge_20261019T084358Z.json`,
sha256 `348a5a4bba1f7a035e47e59d47f2ab2fd38ddb7a05f37d103fa32220d89747e7` (H19:31–32).
None of these verdicts is in E (INV §3, E6).

| # | Item | Verdict | Verdict source |
|---|---|---|---|
| 85 | G-E1, items 1–3 | HOLD | H19:66–73 |
| 86 | P-E1 | 1 component on 5 of 5 cut1200 maps; bound ≥ 4 of 5 | H19:77 |
| 87 | P-E2 | 12 of 12 F\* pairs on 5 of 5 maps; bound ≥ 11 of 12, on ≥ 4 of 5 | H19:78 |
| 88 | P-E3 | 0 maps with a wrong facing edge; bound ≤ 1 | H19:79 |
| 89 | P-E4 | 0 maps with an unmatched facing edge; bound ≤ 1 | H19:80 |

H19:15–16 sums it up: "Gate G-E1 held, and all four registered predictions hold with
no margin used."

### G, steady gate (rows 90–98)

No result file is named, and no sha256 is recorded. G:70 sends output to
`experiments/logs/b18/analysis/`, and H0914:36 records that "`experiments/logs/` is
gitignored". The labels S0–S3 come from H0914, not from G.

| # | Item | Verdict | Verdict source |
|---|---|---|---|
| 90 | V1, retention | passes "hollowly"; rotation retention 23.8 % and 49.7 % (H0914:46) | H0914:52–53 |
| 91 | V2, reduction | passes, hollowly; V2 ratio 3.7 % and 0.3 % (H0914:50) | H0914:52–53 |
| 92 | V3, agreement | "V3 (agreement) still fails" | H0914:69 |
| 93 | V4, flatness | "V4 (flatness) passes at 0.020 and 0.000 points" | H0914:72–73 |
| 94 | V5, contested band (not in G) | FAIL, **INFERRED**. H0914 states how V5 is wired. It does not state that the script exited 2 on run2 or run3. See the quote below. | H0914:59–62 |
| 95 | S0 | pass | H0914:80–81 |
| 96 | S1 | pass | H0914:80–81 |
| 97 | S2 | pass | H0914:80–81 |
| 98 | S3 | "**S3 fails on the synthetic**", "missing the injected `−0.26 %` by 1.638 points" | H0914:75–77 |

**Row 94, quoted.** H0914:59–62: "A verdict that passes because the measurement
collapsed onto an uncontested subset is a failure, and it has to be recordable as one.
**V5 CONTESTED-BAND was added and wired as deciding.** Exit code 2; the script refuses
to hand the mask to B2."

The same table gives, at H0914:49, the retention in the contested 5–45 deg/s band:
"0.2 %" (run2) and "0.0 %" (run3).

### Count

**98 entries.**

| Spec | Rows | Hold or pass | Fail | Void | Controls as intended | No verdict recorded | Reported only |
|---|---|---|---|---|---|---|---|
| W | 39 | 19 | 7 | 3 (T1–T3) | – | 8 (A1–A7, B4) | 2 (C1, C2) |
| D | 36 | 25 | 5 | 3 (P1, P2, P5) | 3 (X1, X6, X7) | – | – |
| S | 9 | 9 | – | – | – | – | – |
| E | 5 | 5 | – | – | – | – | – |
| G | 9 | 6 | 3, of which V5 is INFERRED | – | – | – | – |
| **Total** | **98** | **64** | **15** | **6** | **3** | **8** | **2** |

W's 19 holds include B1 and B2, counted by their final-corpus verdicts. Both FAILED on
the plain corpus (rows 4 and 5).

---

## 3. Errata

The specs stay unedited (§0). Each erratum gives the stale line, what is true now, and
the source. The IDs are INV §3's.

### W

- **W1.**
  - Stale, W:3: "**Status:** v2, after stop point 1. Part A may start once Ali has read
    §V2."
  - True now: Parts A and B are built and validated. W:320 opens "## 11. Choices made
    while writing Part A", and W:374 opens "## 12. What Part B measured, and the two
    predictions that failed".
  - Source: H1004:10–11, "The wall predicate is built and validated (Parts A and B of
    `docs/specs/SPEC_b2_wall_predicate.md`)."
- **W2.**
  - Stale, W:298–302: "1. ~~§1 report.~~ Done." Items 2 and 3 still end "**Stop for
    review.**"
  - True now: both stop points have passed.
  - Source: H1004:10–11.
- **W3.**
  - Stale, W:19: "The handoffs live in the claude.ai project; Ali adds them to
    `docs/handoffs/` separately". W:41 quotes `HANDOFF_phaseB2_schema_entry.md`.
  - True now: the schema-entry handoff is held out of the repo.
  - Source: H19:209–211, "The schema entry handoff, which `SPEC_b2_wall_predicate.md:41`
    quotes, stays out of the repo; §4 restates its plan."; `handoffs_import.md`:36–41.
- **W4.**
  - Stale, W:304–306: "… and sources `/opt/ros/jazzy/setup.bash` and
    `~/Desktop/NSKsim/install/setup.bash`."
  - True now: command blocks source `~/Desktop/NSKsim/ros2_ws/install/setup.bash`
    (H1015:36, H1017:33, H18:22, H19:27).
  - Source: H19:176–177, "the truth run block sourced
    `~/Desktop/NSKsim/install/setup.bash`; chat corrected it to `ros2_ws/install/`
    before the run".
- **W5.**
  - Stale, W:5–8: "Corners, openings, edges, the "same world" statement and the
    divergence measure are later specs. This one records what they will need: endpoint
    states (corners), gap states (openings), viewing directions (corroboration)."
  - True now: D, S and E exist. S reads axis-end states at q0 and q1 (S:117–122) and
    forms its own gaps from axis ends (S:132–137).
  - Source: inv:241–242, "`end_state` is read at `p0`/`p1`, and the axis-end states at
    `q0`/`q1`"; inv:281–282 on graph_walls' `lines[]` gaps, "§4 does not read them."
- **W6.**
  - Stale, W:357–358: "The opening spec will have to say which of the two lengths it
    means;". W:362–363: "The corner spec needs to know this; it is not corrected
    here".
  - True now: no spec has either name. S covers openings and corners, and D-S1 reads
    ends only from axis members (S:29–30, S:97–106).
  - INFERRED that S answers both notes: S does not cite W §11.
- **W7.**
  - Stale, W:804–805: "so that plumbing is Task 2's first step and is not done here."
  - True now: the plumbing was done in Stage F (W:1292–1302). The T1–T3 it served are
    VOID (§4.5).

### D

- **D1.**
  - Stale, D:1 "(v0.22, draft)" and D:3 "Status: draft for review, 4 Oct 2026."
  - True now: P3, P4 and P6 have been "REGISTERED" since v0.6 (D:322–323), and stages
    S5 to S13 ran (D:394–395 to D:1611–1612).
  - Source: H19:200–201.
- **D2.**
  - Stale, D:44–45: "No code is written until the stop-point questions in §11 are
    answered."
  - True now: S1 was answered (D:1736: "Answered 4 Oct 2026; folded into v0.1."), and
    the stages ran (D:1611: "S13 ran at `58b654b`").
- **D3.**
  - Stale, D:1751–1755: S2, S3 and S4 carry no "answered" mark, while S1 does (D:1736).
  - True now: S3 and S4 ran.
  - Source: H1006:79, "Gates (S3, file `s3_gates_20261004T201944Z.json`)"; H1006:88.
- **D4.**
  - Stale, D:69–70: "How the two combine is open (§10)."
  - True now: decided at v0.11. D:455–460: "The two directions, X@a → Y@b and
    Y@b → X@a, are reported side by side with their difference. No mean and no minimum
    is reported". O3 is CLOSED (D:1677).
- **D5.**
  - Stale, D:1727, in O9 (2) (D:1724–1727): "Proposed: yes, to be decided when the
    schema spec is written." (INV §3 cited D:1726–1727.)
  - True now: S:29–30, D-S1, decides it, and the item is closed by ruling (§4.2).
  - Source: H18:164–165.
- **D6.**
  - Stale, D:249: "Per ordered pair (A, B) and cut C (20 pairs × 4 cuts = 80 rows):"
  - True now: D:446, "The full cut matrix has 320 rows: 20 ordered pairs × 4 cuts for
    A × 4 cuts for B." S6 computes all 320 rows (D:467).
- **D7.**
  - Stale: three-class wording outside the T2 record.
    - D:240–242: "so a wall it shadows gets no crossings and falls to unobserved, never
      to contradicted."
    - The breaks X2, X3 and X5 (D:291–294).
    - The synthetic unit cases (D:300–310).
  - True now: two classes since v0.4 (D:65). T2 runs behind a flag that is off by
    default (D:256–257). The code agrees: DW:82 `T2_DEFAULT = False`; DW:83
    `MEASURE_CLASSES = (CORROBORATED, NOT_CORROBORATED)      # v0.4, flag off`.
  - INFERRED stale: §5 and §7 are not marked as T2 records, as §4 marks T2 at D:182.
- **D8.**
  - Stale, D:101–103: "`graph_walls.graph_and_elements(stem)` (`graph_walls.py:1226`)".
    D:148–149: "(`graph_walls.py:1544`)".
  - True now: at HEAD, `graph_and_elements` is defined at `graph_walls.py:1455`, and the
    bearing is computed at `graph_walls.py:1776`. D:124–125's `graph_walls.py:308-313`
    (DIRS) still matches.
  - Source: INV §3 D8, which read `graph_walls.py` at HEAD. No other code line citation
    was checked.
- **D9.**
  - Missing from D: the S3 and S4 results. D mentions them only in passing: D:555 "P3's
    20 series held" and D:1575–1576 "at most 4.1 % in S4's P6 results".
  - True now: G0, G1, X1, X6, P3, P4 and P6 are recorded in H1006:79–94 and
    H1015:205–211, and in register rows 40–43 and 48–50.
- **D10.**
  - Missing from D: G2.
  - True now: RL:185 has "G2: survey the segment guard on the corpus, without reading a
    histogram". H1015:212 has "| G2: histogram join | `25ba70e` | HOLD 20/20 |". DW:64
    holds the tolerance, `SEGMENT_GUARD_TOL_M = 1e-6 # G2: saved vs recomputed segment
    ends`. See register row 51.
- **D11.**
  - Two values disagree. D:200–201 and D:1687 give the deep tail as "4-10 %" of
    crossings. H1006:127–128 gives "on 3–10 % of crossings".
  - The S4 file does not settle it. `s4_predictions_20261005T230035Z.json` (sha256
    `4f173f4c…`, hashed at freeze; block (19 Oct)) opens with the key `"P3"` and its
    rows (lines 1–15). S4 ran with T2 off, and with the flag off "no ray set is built"
    (H1006:67–68), so S4 holds no crossing share. That is INFERRED: the file was not
    read past line 60.
  - **Both values are recorded. The disagreement is unresolved.**

### S

- **S1.**
  - Stale: S:1 "(v0.2, draft)" against S:3 "Status: registered, 17 Oct 2026."
  - True now: S was registered at `5bf3ee2`. The title is kept by decision.
  - Source: H1017:175–176; H19:200–201.
- **S2.**
  - Stale, S:6: "No schema element has been computed on a real map."
  - True now: run 1 computed them on all 20 maps.
  - Source: H18:10–14.
- **S3.**
  - Stale, S:508, the heading "## 11. Open, before the stage".
  - True now: the stage has run.
  - Source: H18:47–49.
- **S4.**
  - Missing from S: the results.
  - True now: register rows 76–84; H18:58–73; result file `84ffdb5a…`.

### E

- **E1.**
  - Stale, E:3: "Status: **v0.2, for registration.** No map data has been read under
    this spec."
  - True now: E was registered at `358650a` (RL:223) and run on the 20 maps on 19 Oct.
  - Source: H19:13–16, H19:57–60.
- **E2.**
  - Missing from E §6 (E:200–210): E-O7.
  - True now: H19:205–206, "**E-O7 (new):** riser transfer at inside corners shortens
    boundary widths and adds no-hit counts. Reported, never gated." It is frozen as a
    limitation (§4.4).
- **E3.**
  - Stale, E:78–80: "**Occluders.** O is the union of: - every segment of the map,
    nodes and excluded segments alike, each as its closed segment p0–p1;"
  - True now: Q1 was accepted. plan:9–10: "**Q1, option (a).** A face's own segment is
    not an occluder for its own rays, and `self_first` is counted."
  - Source: H19:107–109. H19:160–162: "Read literally, §3.3 let a face block its own
    rays: on k0 at least five faces would have lost every sample. The build plan caught
    it (Q1) before any code ran."
- **E4.**
  - Not in E's text, though accepted (plan:8–18). These add to E rather than contradict
    it:
    - **Q7, the strict join** (plan:11–13). E:7 says only "joined by segment `id` and
      `map_sha256`".
    - **Q13, the decoy** (plan:14–15). EJ:126–129 has "Ali's ruling on Q13 (18 Oct
      2026), not registered in the spec", with the decoy face at x = +20. E:181 says
      only "A decoy face placed far from every wall".
    - **Q20, the extra keys** (plan:16–18), against E's output shape at E:131–139.
    - **Q2** (plan:74–76): "Truth must be cast at ρ = 0.1, not at the maps' ρ". EJ:118
      has `TRUTH_RHO = 0.1 # Q2: truth is cast at the rho the spec writes`.
  - INFERRED: further proposals among the 34 may add text that E lacks. The plan's §6
    was not read.
- **E5.**
  - Stale, E:22: "(schema-entry handoff, :133–135)".
  - True now: that handoff is held out of the repo.
  - Source: H19:209–211.
- **E6.**
  - Missing from E: the results.
  - True now: register rows 85–89; H19:66–80; result file `348a5a4b…`.

### G

- **G1.**
  - Stale, G:107–108: "V3 and V4 are informational for B2 — they test whether the gate
    is sound. Only V1 and V2 decide whether B2 can proceed on these terms." G lists
    only V1 to V4 (G:89–105).
  - True now: H0914:59–62, "**V5 CONTESTED-BAND was added and wired as deciding.** Exit
    code 2; the script refuses to hand the mask to B2."
  - Source: H0914:30–34, "V5 and the interval-emitting mask **post-date** the spec".
- **G2.**
  - Stale, G:138–139: "The accepted mask becomes a boolean column carried alongside each
    robot's divergence series."
  - True now: H0914:33 calls it "the interval-emitting mask". H0914:173–174 says "Emit
    masks as time intervals on the source grid, not per-index flags". V5 refuses to
    hand the mask to B2 (H0914:61–62).
- **G3.**
  - Stale, G:131: "The **gated** estimate must recover `-0.26 % ± 0.10` in both mixes."
    G:133–134: "If the gate cannot do this on data with known injected values, it must
    not be pointed at the bags."
  - True now: H0914:75–81, "**S3 fails on the synthetic** … **The default stays 0.60 s
    with the caveat printed.**" Bag results follow at H0914:44–50.
- **G4.**
  - Stale, G:21: "the designed single-robot transient run proposed in §7 of the
    2026-09-13 handoff".
  - True now: amended at H0914:136, "## 6. §7 amended — the designed run". The
    amendments are at H0914:142–151.
- **G5.**
  - Stale, G:136–142 (§8, "How B2 consumes this").
  - True now: no B2 spec consumes G. W, D, S and E do not mention it, and the truth
    uses in D §8 (D:312–318) and S §9 name no mask.
  - Source: INV §3 G5. W, D, S and E were read in full.

---

## 4. Open-item dispositions

Rulings by Ali, 19 Oct 2026, except §4.1, which records closures already in the specs.

### 4.1 Closed in the specs before the freeze (no ruling needed)

| Item | Closed at |
|---|---|
| O1 | D:1666–1669, "CLOSED in v0.10" |
| O3 | D:1677–1679, "CLOSED in v0.20". This includes the open pointer at D:69–70 (erratum D4) |
| O4 | D:1680–1685, "Closed in v0.22" |
| O6 | D:1691–1692, "CLOSED in v0.20" |
| O7 | D:1693–1695, "CLOSED in v0.15" |
| O8 | D:1708–1721, "CLOSED in v0.18" |
| S-O1 | S:510, "Closed in v0.1." |
| S-O3 | S:514, "Closed by §9." |
| S-O4 | S:515, "Closed by §10." |

### 4.2 Closed by ruling (19 Oct)

- **O9 (2)**, D:1724–1727. Closed by S's D-S1 (S:29–30). Already recorded at H1017:192–193
  and H18:164–165.
- **S-O2**, S:511–513. Closed as reported, which is what the spec says: "Reported per
  map through M6, not judged". Run 1 reported one case. H18:88: "One face is split
  across two `lines[]` entries: maze_h2-y on k2 cut60."
- **O9 (1)**, D:1722–1724. Closed, with the measure not changed, because changing it
  would change a registered measure. D:1724: "Changing this would change the
  registered measure, and it is not proposed now."

### 4.3 Moved to B3 (ruling (19 Oct))

- **E-O1**, E:202–203: absolute position in the node features.
- **E-O2**, E:204–205: the feature vector and relation ids. E already says "Belongs to
  the B3 builder" (E:205).
- **E-O5**, E:208–209: how the 6 misaligned slab ends and the 1 unobserved slab end are
  presented.
- **O2**, D:1670–1676: faces with empty bearing histograms.
- **S-O5**, S:516–518: the "in view" rule. Per the ruling, the B3 generator's visibility
  model supplies it. E's earlier-cut reporting depends on S-O5 (E:196–197).

### 4.4 Frozen as registered; carried to Article 3 as limitations (ruling (19 Oct))

- **E-O3**, E:206: unclassified occupied mass does not occlude.
- **E-O4**, E:207: the value of w_min.
- **E-O6**, E:210: a slab end without two points leaves its slab open. On the evidence
  at plan:79–81, "At cut1200 every slab end carries outside corners (40 of 40 in run
  1)".
- **E-O7**, H19:205–206: riser transfer at inside corners.
- **S-O6**, S:519–523: blur beyond A8's tolerance. "Not seen on real maps; margin
  0.0896 m" (H18:163).
- **O5**, D:1686–1690: the deep tail, unexplained. For its share, see erratum D11.
- **O10**, D:1728–1732: `-inf` rays in the offline ray set. H1015:324–326's unchecked
  point stays unchecked.
- **W's spin smear**, W:450–451 and W:667–669: "UNTESTED".

### 4.5 Void (ruling (19 Oct))

- **T1–T3**, W:786–788. Their gated corpus was superseded (H1004:101–103). No verdict was
  ever recorded (register rows 24–26).

### 4.6 Article 3 rule and the steady gate (ruling (19 Oct))

- **The Article 3 rule.** Ruling (19 Oct), verbatim: "The designed run is owed for any
  claim that compares odometry against ground truth. B2's maps were built on known
  poses, so B2's truth validations (P4, A8–A10, the schema matching, the edges judging)
  do not need it."
  - **It narrows H0914:16–17:** "**The designed run of §6 below is now owed for any
    truth-referenced claim in Article 3, not optional.**"
  - **Sources for the ruling's terms:**
    - the designed run, H0914:136–151;
    - the known poses, H1004:69, "Known-pose offline mapping";
    - P4, D:337–339;
    - A8–A10, W:230–232;
    - the schema matching, S:248–406;
    - the edges judging, E:183–194.
- **No B2 spec consumes the steady gate.** See erratum G5.
- **The rise/fall asymmetry stays provisional.** H0914:133–134: "It stays provisional,
  and it cannot be resolved from these bags".

---

## 5. Dropped on the record (ruling (19 Oct))

- **w_null and E7 at mixed cuts.** They were never computed and never registered. D:1346–1347:
  "**What is not computed.** w_null and E7 at mixed cuts. S12 does not compute them, so a
  prediction about them can still be registered." They were carried at H1015:327–328
  and H18:169–170.
- **Listed as a candidate prediction for B4, on fresh data.**

---

## 6. Carried, not spec items

Every destination below is set by ruling (19 Oct).

| Item | Source | Destination |
|---|---|---|
| The S9–S11 fail paths | H1015:330–331, "Fix them in their own commit before any of those stages is rerun."; D:1551–1553 | **Fix before any rerun** |
| The explorer and mapper defects | H1004:191–195; H19:99–100, "The explorer and mapper defect work waits until B3 is done."; H19:213–214, "(after B3)" | **Before the B3 test recordings.** This moves the work ahead of the B3 test recordings; H19 had put it after B3. |
| Extent boxes not nested across cuts | H1015:335; H18:171 | **With the explorer and mapper fixes, before the B3 test recordings** |
| W's B3 not judged on `b2maps_k4_cut1200_gated_extfix_robot4` | W:1329–1332; H1004:243–244, "Check the stopper's grace before the next batch." (also H1015:336–337) | **With the explorer and mapper fixes, before the B3 test recordings**, as a check of the stopper's grace |
| The pitched scan plane | D:1599–1607; H1015:332, "belongs in Article 3's description of the sensor." | **Article 3** |
| The fixed-A weight series | D:1423–1428; H1015:329, "How Article 3 shows and reads them is open." | **Article 3** |
| The three unchecked points of the 13 Oct handoff | H1015:286–289 | **Checked before Article 3 cites them** |
| Claude Code's auto-mode allow list | H1015:339–340, "check that Claude Code's auto mode keeps git commit, git push and simulation launches out of its allow list." | **Checked at the start of the next fresh Claude Code session** |

The three unchecked points, as H1015:286–289 states them:
1. §4.4's "b sat at an end of its run" (second and second-to-last are not ends);
2. §4.4's largest dot at k′ (0.0197, a tilt near 1.13°) against §6's two tilts above
   1° and §7's +1.0104° at k′;
3. CI for `ed2a618`, never recorded.

**Scratch removal stays out** (ruling (19 Oct)). H18:173 lists "scratch to remove from
~/Downloads/". It is not carried here.
