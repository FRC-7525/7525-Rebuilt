import sys
import re
from pathlib import Path
from graphviz import Digraph

# ---------- Parsing ----------

def get_states(path):
    text = Path(path).read_text(encoding="utf8")
    return re.findall(r"\w+\s*\((?:[\s\S]*?)\)[,;]", text)


def parse_states(blocks):
    states = []

    for block in blocks:
        name_match = re.match(r"(\w+)\s*\(", block)
        if not name_match:
            continue

        name = name_match.group(1)
        subs = dict(
            (k.replace("States", ""), v)
            for k, v in re.findall(r"(\w+States)\.(\w+)", block)
        )

        states.append({
            "stateName": name,
            "Intake": subs.get("Intake", "—"),
            "Hopper": subs.get("Hopper", "—"),
            "Shooter": subs.get("Shooter", "—"),
            "Climber": subs.get("Climber", "—"),
        })

    return states


def get_triggers(path):
    text = Path(path).read_text(encoding="utf8")
    return re.findall(r"addTrigger\([\s\S]*?\);", text)


def parse_triggers(triggers):
    parsed = []

    for t in triggers:
        m = re.search(
            r"""
            addTrigger\(
                \s*(?:\w+\.)?(\w+)\s*,       # from
                \s*(?:\w+\.)?(\w+)\s*,       # to
                \s*(.+?)                     # condition (full)
            \)
            """,
            t,
            re.VERBOSE | re.DOTALL,
        )

        if not m:
            continue

        cond = m.group(3).strip()

        # Clean lambda syntax
        cond = re.sub(r"^\(\)\s*->\s*", "", cond)

        # Remove trailing semicolon if present
        cond = cond.rstrip(";")

        parsed.append({
            "from": m.group(1),
            "to": m.group(2),
            "condition": cond,
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


def nid(name):
    return name.replace("-", "_")


def state_label(state, info):
    return f"""<<TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="6" BGCOLOR="#1e293b">
        <TR>
            <TD ALIGN="CENTER">
                <B><FONT COLOR="white">{state}</FONT></B><BR/>
                <FONT COLOR="#cbd5e1" POINT-SIZE="10">
                    Intake: {info["Intake"]}<BR/>
                    Hopper: {info["Hopper"]}<BR/>
                    Shooter: {info["Shooter"]}<BR/>
                    Climber: {info["Climber"]}
                </FONT>
            </TD>
        </TR>
    </TABLE>>"""


def edge_label(text):
    return f"""<<TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="4" BGCOLOR="#020617">
        <TR>
            <TD><FONT COLOR="white" POINT-SIZE="10">{text}</FONT></TD>
        </TR>
    </TABLE>>"""


def generate_graph(state_map):
    dot = Digraph("StateMachine", engine="dot", format="png")

    dot.attr(
        bgcolor="#020617",
        rankdir="TB",
        splines="polyline",
        nodesep="0.6",
        ranksep="0.9",
        fontname="Helvetica"
    )

    # Nodes
    for state, info in state_map.items():
        dot.node(
            nid(state),
            label=state_label(state, info),
            shape="plaintext"
        )

    # Edges
    color_i = 0
    for state, info in state_map.items():
        connections = info.get("connections", [])
        if not connections:
            continue

        color = EDGE_COLORS[color_i % len(EDGE_COLORS)]
        color_i += 1

        for c in connections:
            dot.edge(
                nid(state),
                nid(c["to"]),
                label=edge_label(c["condition"]),
                color=color,
                penwidth="1.6",
                fontcolor="white"
            )

    dot.render("state_machine", cleanup=True)
    print("✅ state_machine.png generated")


# ---------- Main ----------

def main(states_path, triggers_path):
    states = parse_states(get_states(states_path))
    triggers = parse_triggers(get_triggers(triggers_path))
    state_map = create_state_map(states, triggers)
    generate_graph(state_map)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
