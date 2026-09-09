#!/usr/bin/env python3
"""
Public-facing NSKsim overview figure — for the GitHub README and a CV portfolio.

Nothing here comes from the internal handoffs' strategy, supervisor, publication
or career threads. Nine cards, one page, readable at thumbnail size.

Usage:  python3 make_public_overview.py
Output: nsksim_overview_light.svg, nsksim_overview_dark.svg
"""

W, H = 1570, 812

THEMES = {
    "light": dict(
        bg="#ffffff", card="#ffffff", ink="#12161c", mute="#5c6570",
        body="#333b45", rule="#dfe3e8", spine="#c3cad2",
        band="#f6f7f9",
        acc={"built": "#2e7d47", "meas": "#3355a6", "open": "#b03a30",
             "method": "#5c6570"},
        tagbg={"built": "#e8f3ea", "meas": "#eaeefa", "open": "#fbeceb",
               "method": "#eef0f2"},
    ),
    "dark": dict(
        bg="#0d1117", card="#161b22", ink="#e6edf3", mute="#8b949e",
        body="#c9d1d9", rule="#30363d", spine="#30363d",
        band="#11161d",
        acc={"built": "#4ec97a", "meas": "#7aa2f7", "open": "#f0776a",
             "method": "#8b949e"},
        tagbg={"built": "#132a1c", "meas": "#141d33", "open": "#2b1614",
               "method": "#1b2027"},
    ),
}

TITLE = "NSKsim — multi-robot ontology convergence"
SUB = ("ROS 2 Jazzy · Gazebo Harmonic · Nav2 · slam_toolbox · "
       "5 × TurtleBot3 Burger · 20 × 20 m world")
LEDE = ("How a group of robots with divergent, viewpoint-dependent views of the "
        "same environment converges on one shared map of it.")

CARDS = [
    ("1", "PLATFORM", "built", "BUILT",
     "Five TurtleBot3 Burger robots with full differential-drive physics in a "
     "20 × 20 m Gazebo world. Namespaced ROS 2 graph, per-robot models built at "
     "launch, Gazebo clock bridged so every node runs on simulation time."),

    ("2", "AUTONOMOUS EXPLORATION", "built", "BUILT",
     "slam_toolbox and Nav2 per robot, driven by a custom frontier explorer. A "
     "robot maps the maze unattended at 0.1 m resolution — no scripted waypoints "
     "anywhere in the loop."),

    ("3", "RELIABLE TERMINATION", "meas", "MEASURED",
     "Outcome classification, reachability pre-check, bounded retry ladder and a "
     "goal supervisor that works in simulation time. First clean exit on frontier "
     "exhaustion: 172 goals, no intervention."),

    ("4", "MULTI-ROBOT EFFECTS", "meas", "DIAGNOSED",
     "Peer robots are written permanently into each other's occupancy grid as "
     "static obstacles, which can leave a robot's own start cell unplannable. "
     "Found by reading the planner's costmap live."),

    ("5", "FIVE STACKS CONCURRENT", "built", "BUILT",
     "All five Nav2 + SLAM stacks active together on a 4-core laptop under paced "
     "bringup. The ceiling is lifecycle service latency under CPU starvation, not "
     "any configured timeout."),

    ("6", "RECORDED RUN", "built", "BUILT",
     "A 2.3 GiB, 3.4-million-message bag of five robots exploring at once: "
     "per-robot scans, odometry and transforms, stamped in simulation time and "
     "replayable end to end."),

    ("7", "OFFLINE MAPPING PIPELINE", "built", "BUILT",
     "Deterministic offline SLAM replay per robot, plus a fitter that places each "
     "map into world coordinates and checks the free fit against an analytic "
     "transform. Verified at 97 % on a known-good map."),

    ("8", "OPEN — WHERE IT STANDS", "open", "CURRENT",
     "No map from the five-robot run clears the 80 % fit floor: each is the same "
     "walls drawn two or three times, rotated. Three explanations eliminated by "
     "measurement; the next run records ground-truth pose."),

    ("", "HOW IT IS BUILT", "method", "METHOD",
     "Every claim is verified at runtime before it is committed. Refuted "
     "hypotheses and negative results are written into the repository rather than "
     "discarded — 20 session records, 204 tests green."),
]

FOOT_L = "github.com/AliAlhasan6/NSKsim"
FOOT_R = "July – September 2026"


def wrap(txt, px, size, maxlines=4):
    """Greedy wrap using a conservative per-character width (DejaVu-safe)."""
    cw = size * 0.545
    limit = int(px / cw)
    words, lines, cur = txt.split(), [], ""
    for w in words:
        trial = w if not cur else cur + " " + w
        if len(trial) <= limit:
            cur = trial
        else:
            lines.append(cur)
            cur = w
    if cur:
        lines.append(cur)
    return lines[:maxlines]


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def build(theme_name):
    T = THEMES[theme_name]
    o = []
    def add(s): o.append(s)

    SANS = "Inter, 'Helvetica Neue', Helvetica, Arial, sans-serif"
    MONO = "ui-monospace, 'SF Mono', Menlo, Consolas, monospace"

    def text(x, y, s, size=13, fill=None, weight="400", anchor="start",
             family=SANS, spacing=None):
        ls = f' letter-spacing="{spacing}"' if spacing else ""
        add(f'<text x="{x}" y="{y}" font-family="{family}" font-size="{size}" '
            f'fill="{fill or T["ink"]}" font-weight="{weight}" '
            f'text-anchor="{anchor}"{ls}>{esc(s)}</text>')

    def rect(x, y, w, h, fill, stroke=None, rx=10, sw=1):
        s = f' stroke="{stroke}" stroke-width="{sw}"' if stroke else ""
        add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" '
            f'fill="{fill}"{s}/>')

    add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
        f'viewBox="0 0 {W} {H}">')
    rect(0, 0, W, H, T["bg"], rx=0)

    # header
    M = 60
    text(M, 66, TITLE, size=30, weight="700")
    text(M, 94, SUB, size=13.5, fill=T["mute"], family=MONO)
    text(M, 126, LEDE, size=14.5, fill=T["body"])
    add(f'<line x1="{M}" y1="150" x2="{W-M}" y2="150" '
        f'stroke="{T["rule"]}" stroke-width="1"/>')

    # grid
    COLS, CW, CH, GAP = 3, 463, 152, 26
    top = 176
    for i, (num, title, key, taglabel, body) in enumerate(CARDS):
        c, r = i % COLS, i // COLS
        x = M + c * (CW + GAP)
        y = top + r * (CH + GAP)
        acc = T["acc"][key]
        is_method = key == "method"

        rect(x, y, CW, CH, T["band"] if is_method else T["card"],
             T["rule"], rx=12)
        add(f'<path d="M {x} {y+12} Q {x} {y} {x+12} {y} L {x+12} {y} '
            f'L {x+12} {y+CH} L {x+12} {y+CH} Q {x} {y+CH} {x} {y+CH-12} Z" '
            f'fill="{acc}"/>')

        if num:
            add(f'<circle cx="{x+48}" cy="{y+34}" r="14" fill="none" '
                f'stroke="{acc}" stroke-width="1.4"/>')
            text(x + 48, y + 39, num, size=13.5, fill=acc, weight="700",
                 anchor="middle")
            tx = x + 74
        else:
            tx = x + 32

        text(tx, y + 39, title, size=14, weight="700", spacing="0.6")

        tw = 11 + len(taglabel) * 6.4
        rect(x + CW - 20 - tw, y + 25, tw, 18, T["tagbg"][key], acc, rx=9)
        text(x + CW - 20 - tw / 2, y + 37.5, taglabel, size=9, fill=acc,
             weight="700", anchor="middle", spacing="0.8")

        for j, ln in enumerate(wrap(body, CW - 64, 12.4)):
            text(x + 32, y + 68 + j * 19, ln, size=12.4, fill=T["body"])

    # footer
    fy = top + 3 * (CH + GAP) + 4
    add(f'<line x1="{M}" y1="{fy}" x2="{W-M}" y2="{fy}" '
        f'stroke="{T["rule"]}" stroke-width="1"/>')
    text(M, fy + 26, FOOT_L, size=12.5, fill=T["mute"], family=MONO)
    text(W - M, fy + 26, FOOT_R, size=12.5, fill=T["mute"], anchor="end")

    add("</svg>")
    return "\n".join(o)


for name in ("light", "dark"):
    path = f"/home/claude/nsksim_overview_{name}.svg"
    with open(path, "w") as f:
        f.write(build(name))
    print("wrote", path)
