import sys
import re
from pathlib import Path
from graphviz import Digraph

EXCLUDED_STATES = []

# ---------- Parsing ----------
def get_states(manager_states_path):
    file = Path(manager_states_path).read_text(encoding="utf8")
    return re.findall(r"\s*(?:\w+)\s*\((?:[\s\S]*?)\)[,;]", file)

def parse_states(states):
    parsed = []
    for state in states:
        name = re.match(r"^\s*(\w+)\s*\(", state)
        parsed.append({
            "stateName": name[1] if name else None,
            "subStates": re.findall(r"\b(?:\w+\.)+\w+\b", state)
        })
    return parsed

def get_triggers(manager_path):
    file = Path(manager_path).read_text(encoding="utf8")
    return re.findall(r"addTrigger\(.+?\)", file)

def parse_triggers(triggers):
    parsed = []
    for t in triggers:
        # Strip enum prefixes for states
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
        if m:
            condition = m[3]
            # Remove everything before 'get' or last :: for brevity
            condition = re.sub(r".*::get", "", condition)
            condition = re.sub(r".*::", "", condition)
            parsed.append({
                "from": m[1],
                "to": m[2],
                "condition": condition
            })
    return parsed

def create_state_map(states, triggers):
    state_map = {s["stateName"]: s for s in states}
    for t in triggers:
        state = state_map.get(t["from"])
        if state is not None:
            state.setdefault("connectionsTo", []).append(t)
    return state_map

# ---------- Graphviz helpers ----------
def node_id(name: str) -> str:
    """Sanitize node names for Graphviz IDs."""
    return re.sub(r"\W+", "_", name)

# Generate a color for each node’s outgoing edges
EDGE_COLORS = [
    "#f6e05e", "#68d391", "#63b3ed", "#fc8181",
    "#90cdf4", "#faf089", "#fbb6ce", "#9f7aea"
]

# ---------- Visualization ----------
def generate_graph(state_map):
    dot = Digraph(
        "StateMachine",
        format="png",
        engine="dot",
    )

    # General graph styling (dark mode)
    dot.attr(
        rankdir="TB",
        bgcolor="#1e1e2f",
        fontname="Helvetica",
    )
    dot.attr(
        "node",
        shape="rounded",
        style="filled",
        fontname="Helvetica-Bold",
        fontsize="11",
        fillcolor="#2d2d3c",
        color="#68d391",
        fontcolor="white",
        penwidth="1.5",
    )
    dot.attr(
        "edge",
        fontname="Helvetica",
        fontsize="9",
        color="#ffffff",
        arrowsize="0.8",
    )

    # Collect all nodes: states + any targets from triggers
    all_nodes = set(state_map.keys())
    for info in state_map.values():
        for c in info.get("connectionsTo", []):
            all_nodes.add(c["to"])

    # Add nodes
    for state in all_nodes:
        if state == "IDLE":
            dot.node(node_id(state), label=state,
                     fillcolor="#3e2c1c",
                     color="#f6ad55",
                     fontcolor="white",
                     penwidth="2.5")
        else:
            dot.node(node_id(state), label=state)

    # Add edges, colored per source node
    color_index = 0
    for state, info in state_map.items():
        if not info.get("connectionsTo"):
            continue
        color = EDGE_COLORS[color_index % len(EDGE_COLORS)]
        color_index += 1
        for c in info.get("connectionsTo", []):
            dot.edge(
                node_id(state),
                node_id(c["to"]),
                label=c["condition"],
                color=color,
                fontcolor=color,
                penwidth="1.8",
                constraint="false",  # makes arrows slightly curved, easier to follow
            )

    # Render
    dot.render("state_machine", cleanup=True)
    print("Rendered state_machine.png")

# ---------- Main ----------
def main(managerStatesPath, managerPath):
    states = parse_states(get_states(managerStatesPath))
    triggers = parse_triggers(get_triggers(managerPath))
    state_map = create_state_map(states, triggers)

    # Debug: show parsed triggers
    print("Triggers found:")
    for t in triggers:
        print(f"{t['from']} -> {t['to']} [{t['condition']}]")

    generate_graph(state_map)
    print("Success!")

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
