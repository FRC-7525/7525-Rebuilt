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
    return re.findall(r"addTrigger\([\s\S]*?\);", file)

def parse_triggers(triggers):
    parsed = []

    for t in triggers:
        m = re.search(
            r"""
            addTrigger\(
                \s*(?:\w+\.)?(\w+)\s*,      # from
                \s*(?:\w+\.)?(\w+)\s*,      # to
                \s*(.*?)                    # condition (FULL)
            \)\s*;
            """,
            t,
            re.VERBOSE | re.DOTALL,
        )
        if not m:
            continue

        condition = m.group(3).strip()

        # Remove lambda wrapper
        condition = re.sub(r"^\(\)\s*->\s*", "", condition)

        # Remove controller prefixes
        condition = re.sub(r"(?:DRIVER|OPERATOR)_CONTROLLER::get", "", condition)
        condition = re.sub(r".*?::", "", condition)

        parsed.append({
            "from": m.group(1),
            "to": m.group(2),
            "condition": condition,
        })

    return parsed

def attach_triggers(state_map, triggers):
    for t in triggers:
        if t["from"] in state_map:
            state_map[t["from"]]["connectionsTo"].append(t)

# ---------- Graphviz helpers ----------
def node_id(name: str) -> str:
    return re.sub(r"\W+", "_", name)

EDGE_COLORS = [
    "#f6e05e", "#68d391", "#63b3ed", "#fc8181",
    "#90cdf4", "#faf089", "#fbb6ce", "#9f7aea"
]

# ---------- Visualization ----------
def generate_graph(state_map):
    dot = Digraph("StateMachine", format="png", engine="dot")

    dot.attr(
        rankdir="TB",
        bgcolor="#1e1e2f",
        fontname="Helvetica",
        nodesep="0.6",
        ranksep="0.9",
    )

    dot.attr(
        "node",
        shape="box",
        style="rounded,filled",
        fillcolor="#2d2d3c",
        color="#68d391",
        fontcolor="white",
        fontname="Helvetica-Bold",
        fontsize="11",
        penwidth="1.4",
    )

    dot.attr(
        "edge",
        fontname="Helvetica",
        fontsize="9",
        arrowsize="0.8",
    )

    # Nodes
    for state, info in state_map.items():
        lines = [
            f"{state}",
            "────────────",
            f"Intake   → {info['Intake']}",
            f"Hopper   → {info['Hopper']}",
            f"Shooter  → {info['Shooter']}",
            f"Climber  → {info['Climber']}",
        ]
        label = "\n".join(lines)

        if state == "IDLE":
            dot.node(
                node_id(state),
                label=label,
                fillcolor="#3e2c1c",
                color="#f6ad55",
                penwidth="2.5",
            )
        else:
            dot.node(node_id(state), label=label)

    # Edges
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
