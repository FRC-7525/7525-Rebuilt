import sys
import re
from pathlib import Path
from graphviz import Digraph

# ---------- Parsing ----------

def get_states(path):
    return re.findall(
        r"\w+\s*\((?:[\s\S]*?)\)[,;]",
        Path(path).read_text(encoding="utf8")
    )


def parse_states(blocks):
    states = []
    for b in blocks:
        name = re.match(r"(\w+)\s*\(", b)
        if not name:
            continue

        subs = dict(
            (k.replace("States", ""), v)
            for k, v in re.findall(r"(\w+States)\.(\w+)", b)
        )

        states.append({
            "stateName": name.group(1),
            **subs
        })

    return states


def get_triggers(path):
    return re.findall(
        r"addTrigger\([\s\S]*?\)",
        Path(path).read_text(encoding="utf8")
    )


def parse_triggers(triggers):
    parsed = []
    for t in triggers:
        m = re.search(
            r"addTrigger\(\s*(?:\w+\.)?(\w+)\s*,\s*(?:\w+\.)?(\w+)\s*,\s*([^)]+)\)",
            t
        )
        if not m:
            continue

        cond = m.group(3)
        cond = re.sub(r".*::get", "", cond)
        cond = cond.replace("()", "")

        parsed.append({
            "from": m.group(1),
            "to": m.group(2),
            "condition": cond
        })

    return parsed


def create_state_map(states, triggers):
    sm = {s["stateName"]: s for s in states}
    for t in triggers:
        sm[t["from"]].setdefault("connections", []).append(t)
    return sm


# ---------- Graph ----------

EDGE_COLORS = [
    "#38bdf8", "#4ade80", "#f472b6",
    "#facc15", "#a78bfa", "#fb7185"
]


def nid(s):
    return s.replace("-", "_")


def generate_graph(state_map):
    dot = Digraph("StateMachine", engine="dot", format="png")

    dot.attr(
        bgcolor="#020617",
        rankdir="TB",
        splines="polyline",   # IMPORTANT
        nodesep="0.6",
        ranksep="0.8",
        fontname="Helvetica"
    )

    # Nodes
    for state, info in state_map.items():
        label = f"""<<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="6" BGCOLOR="#1e293b">
            <TR>
                <TD COLSPAN="2" BGCOLOR="#334155">
                    <B><FONT COLOR="white">{state}</FONT></B>
                </TD>
            </TR>
            <TR><TD><FONT COLOR="#cbd5e1">Intake</FONT></TD><TD><FONT COLOR="white">{info.get("Intake","—")}</FONT></TD></TR>
            <TR><TD><FONT COLOR="#cbd5e1">Hopper</FONT></TD><TD><FONT COLOR="white">{info.get("Hopper","—")}</FONT></TD></TR>
            <TR><TD><FONT COLOR="#cbd5e1">Shooter</FONT></TD><TD><FONT COLOR="white">{info.get("Shooter","—")}</FONT></TD></TR>
            <TR><TD><FONT COLOR="#cbd5e1">Climber</FONT></TD><TD><FONT COLOR="white">{info.get("Climber","—")}</FONT></TD></TR>
        </TABLE>>"""

        dot.node(nid(state), label=label, shape="plaintext")

    # Edges
    color_i = 0
    for s, info in state_map.items():
        if "connections" not in info:
            continue

        color = EDGE_COLORS[color_i % len(EDGE_COLORS)]
        color_i += 1

        for c in info["connections"]:
            edge_label = f"""<<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4" BGCOLOR="#020617">
                <TR>
                    <TD><FONT COLOR="white">{c["condition"]}</FONT></TD>
                </TR>
            </TABLE>>"""

            dot.edge(
                nid(s),
                nid(c["to"]),
                label=edge_label,   # NOT xlabel
                color=color,
                fontcolor="white",
                penwidth="1.6"
            )

    dot.render("state_machine", cleanup=True)
    print("✅ state_machine.png generated")


# ---------- Main ----------

def main(states_path, triggers_path):
    states = parse_states(get_states(states_path))
    triggers = parse_triggers(get_triggers(triggers_path))
    generate_graph(create_state_map(states, triggers))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
