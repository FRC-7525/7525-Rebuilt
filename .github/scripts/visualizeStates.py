import sys
import re
from pathlib import Path
from graphviz import Digraph

# ---------- Parsing ManagerStates ----------
def get_manager_states(manager_states_path):
    text = Path(manager_states_path).read_text(encoding="utf8")

    pattern = re.compile(
        r"""
        (\w+)\s*
        \(\s*
            "(.*?)"\s*,\s*
            ([^,]+)\s*,\s*
            ([^,]+)\s*,\s*
            ([^,]+)\s*,\s*
            ([^)]+)
        \)
        """,
        re.VERBOSE | re.DOTALL,
    )

    states = {}

    for m in pattern.finditer(text):
        name = m.group(1)

        def clean(x: str) -> str:
            x = x.strip()
            x = re.sub(r".*?\.", "", x)
            x = re.sub(r"\(.*\)", "", x)
            return x

        states[name] = {
            "Intake": clean(m.group(3)),
            "Hopper": clean(m.group(4)),
            "Shooter": clean(m.group(5)),
            "Climber": clean(m.group(6)),
            "connectionsTo": [],
        }

    return states

# ---------- Parsing Triggers ----------
def get_triggers(manager_path):
    file = Path(manager_path).read_text(encoding="utf8")
    return re.findall(r"addTrigger\(.+?\)", file)

def parse_triggers(triggers):
    parsed = []
    for t in triggers:
        m = re.search(
            r"""
            addTrigger\(
                \s*(?:\w+\.)?(\w+)\s*,
                \s*(?:\w+\.)?(\w+)\s*,
                \s*([^)\s]+)
            \)
            """,
            t,
            re.VERBOSE,
        )
        if m:
            condition = m[3]
            condition = re.sub(r".*::get", "", condition)
            condition = re.sub(r".*::", "", condition)
            parsed.append({
                "from": m[1],
                "to": m[2],
                "condition": condition,
            })
    return parsed

def attach_triggers(state_map, triggers):
    for t in triggers:
        if t["from"] in state_map:
            state_map[t["from"]]["connectionsTo"].append(t)

# ---------- Helpers ----------
def node_id(name: str) -> str:
    return re.sub(r"\W+", "_", name)

EDGE_COLORS = [
    "#f6e05e", "#68d391", "#63b3ed", "#fc8181",
    "#90cdf4", "#faf089", "#fbb6ce", "#9f7aea"
]

# ---------- Visualization ----------
def generate_graph(state_map):
    dot = Digraph("StateMachine", format="png", engine="dot")

    # Graph-level styling
    dot.attr(
        rankdir="TB",
        bgcolor="#1e1e2f",
        fontname="Helvetica",
        nodesep="0.6",
        ranksep="0.9",
        splines="ortho",          # ✅ 90-degree edges
        concentrate="false",
    )

    dot.attr(
        "edge",
        fontname="Helvetica",
        fontsize="9",
        arrowsize="0.8",
    )

    # Nodes with HTML labels
    for state, info in state_map.items():
        label = f"""
        <
        <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="6" BGCOLOR="#2d2d3c">
            <TR>
                <TD COLSPAN="2" BGCOLOR="#3b3b4f">
                    <B><FONT COLOR="white">{state}</FONT></B>
                </TD>
            </TR>
            <TR><TD ALIGN="LEFT"><FONT COLOR="#cbd5e0">Intake</FONT></TD><TD ALIGN="LEFT"><FONT COLOR="white">{info['Intake']}</FONT></TD></TR>
            <TR><TD ALIGN="LEFT"><FONT COLOR="#cbd5e0">Hopper</FONT></TD><TD ALIGN="LEFT"><FONT COLOR="white">{info['Hopper']}</FONT></TD></TR>
            <TR><TD ALIGN="LEFT"><FONT COLOR="#cbd5e0">Shooter</FONT></TD><TD ALIGN="LEFT"><FONT COLOR="white">{info['Shooter']}</FONT></TD></TR>
            <TR><TD ALIGN="LEFT"><FONT COLOR="#cbd5e0">Climber</FONT></TD><TD ALIGN="LEFT"><FONT COLOR="white">{info['Climber']}</FONT></TD></TR>
        </TABLE>
        >
        """

        if state == "IDLE":
            dot.node(
                node_id(state),
                label=label,
                shape="plaintext",
                color="#f6ad55",
            )
        else:
            dot.node(
                node_id(state),
                label=label,
                shape="plaintext",
            )

    # Edges (colored per source)
    color_index = 0
    for state, info in state_map.items():
        if not info["connectionsTo"]:
            continue

        color = EDGE_COLORS[color_index % len(EDGE_COLORS)]
        color_index += 1

        for c in info["connectionsTo"]:
            dot.edge(
                node_id(state),
                node_id(c["to"]),
                label=c["condition"],
                color=color,
                fontcolor=color,
                penwidth="1.8",
            )

    dot.render("state_machine", cleanup=True)
    print("Rendered state_machine.png")

# ---------- Main ----------
def main(managerStatesPath, managerPath):
    state_map = get_manager_states(managerStatesPath)
    triggers = parse_triggers(get_triggers(managerPath))
    attach_triggers(state_map, triggers)

    generate_graph(state_map)
    print("Success!")

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
