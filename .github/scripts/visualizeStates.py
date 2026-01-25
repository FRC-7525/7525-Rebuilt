import sys
import re
from pathlib import Path
from graphviz import Digraph

# ---------- Parsing ----------

def get_states(manager_states_path):
    file = Path(manager_states_path).read_text(encoding="utf8")
    return re.findall(r"\w+\s*\((?:[\s\S]*?)\)[,;]", file)


def parse_states(states):
    parsed = []

    for block in states:
        name_match = re.match(r"(\w+)\s*\(", block)
        if not name_match:
            continue

        name = name_match.group(1)

        subs = re.findall(r"(\w+States)\.(\w+)", block)
        sub_map = {k.replace("States", ""): v for k, v in subs}

        parsed.append({
            "stateName": name,
            "Intake": sub_map.get("Intake", "—"),
            "Hopper": sub_map.get("Hopper", "—"),
            "Shooter": sub_map.get("Shooter", "—"),
            "Climber": sub_map.get("Climber", "—"),
        })

    return parsed


def get_triggers(manager_path):
    file = Path(manager_path).read_text(encoding="utf8")
    return re.findall(r"addTrigger\([\s\S]*?\)", file)


def parse_triggers(triggers):
    parsed = []

    for t in triggers:
        m = re.search(
            r"""
            addTrigger\(
                \s*(?:\w+\.)?(\w+)\s*,      # from
                \s*(?:\w+\.)?(\w+)\s*,      # to
                \s*([^)\s]+)                # condition
            \)
            """,
            t,
            re.VERBOSE,
        )
        if not m:
            continue

        condition = m.group(3)
        condition = re.sub(r".*::get", "", condition)
        condition = re.sub(r"\(\)", "", condition)

        parsed.append({
            "from": m.group(1),
            "to": m.group(2),
            "condition": condition,
        })

    return parsed


def create_state_map(states, triggers):
    state_map = {s["stateName"]: s for s in states}
    for t in triggers:
        state_map.setdefault(t["from"], {}).setdefault("connectionsTo", []).append(t)
    return state_map


# ---------- GraphViz ----------

EDGE_COLORS = [
    "#60a5fa", "#34d399", "#f472b6",
    "#fbbf24", "#a78bfa", "#fb7185"
]


def node_id(name):
    return name.replace("-", "_")


def generate_graph(state_map):
    dot = Digraph(
        "StateMachine",
        engine="dot",
        format="png",
    )

    dot.attr(
        bgcolor="#0f172a",
        rankdir="TB",
        splines="ortho",
        nodesep="0.6",
        ranksep="0.8",
        fontname="Helvetica"
    )

    # ----- Nodes -----
    for state, info in state_map.items():
        label = f"""<<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="6" BGCOLOR="#1e293b">
            <TR>
                <TD COLSPAN="2" BGCOLOR="#334155">
                    <B><FONT COLOR="white">{state}</FONT></B>
                </TD>
            </TR>
            <TR><TD ALIGN="LEFT"><FONT COLOR="#cbd5e1">Intake</FONT></TD><TD><FONT COLOR="white">{info.get("Intake","—")}</FONT></TD></TR>
            <TR><TD ALIGN="LEFT"><FONT COLOR="#cbd5e1">Hopper</FONT></TD><TD><FONT COLOR="white">{info.get("Hopper","—")}</FONT></TD></TR>
            <TR><TD ALIGN="LEFT"><FONT COLOR="#cbd5e1">Shooter</FONT></TD><TD><FONT COLOR="white">{info.get("Shooter","—")}</FONT></TD></TR>
            <TR><TD ALIGN="LEFT"><FONT COLOR="#cbd5e1">Climber</FONT></TD><TD><FONT COLOR="white">{info.get("Climber","—")}</FONT></TD></TR>
        </TABLE>>"""

        dot.node(
            node_id(state),
            label=label,
            shape="plaintext"
        )

    # ----- Edges -----
    color_index = 0
    for state, info in state_map.items():
        connections = info.get("connectionsTo", [])
        if not connections:
            continue

        color = EDGE_COLORS[color_index % len(EDGE_COLORS)]
        color_index += 1

        for c in connections:
            edge_label = f"""<<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4" BGCOLOR="#020617">
                <TR>
                    <TD><FONT COLOR="white">{c["condition"]}</FONT></TD>
                </TR>
            </TABLE>>"""

            dot.edge(
                node_id(state),
                node_id(c["to"]),
                color=color,
                penwidth="1.8",
                xlabel=edge_label,
            )

    dot.render("state_machine", cleanup=True)
    print("✅ state_machine.png generated")


# ---------- Main ----------

def main(managerStatesPath, managerPath):
    states = parse_states(get_states(managerStatesPath))
    triggers = parse_triggers(get_triggers(managerPath))
    state_map = create_state_map(states, triggers)
    generate_graph(state_map)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
