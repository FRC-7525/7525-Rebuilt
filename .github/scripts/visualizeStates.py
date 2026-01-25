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
        # Improved regex: strips enum prefixes like ManagerStates.
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
            parsed.append({
                "from": m[1],
                "to": m[2],
                "condition": m[3]
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

# ---------- Visualization ----------
def generate_graph(state_map):
    dot = Digraph(
        "StateMachine",
        format="png",
        engine="dot",
    )

    # General graph styling
    dot.attr(
        rankdir="TB",
        bgcolor="#f8f9fb",
        fontname="Helvetica",
    )
    dot.attr(
        "node",
        shape="rounded",
        style="filled",
        fontname="Helvetica-Bold",
        fontsize="11",
        fillcolor="#cfe8ff",
        color="#2b6cb0",
        penwidth="1.5",
    )
    dot.attr(
        "edge",
        fontname="Helvetica",
        fontsize="9",
        color="#4a5568",
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
                     fillcolor="#ffe8cc",
                     color="#c05621",
                     penwidth="2.5")
        else:
            dot.node(node_id(state), label=state)

    # Add edges
    for state, info in state_map.items():
        for c in info.get("connectionsTo", []):
            label = c["condition"].replace("->", "→")
            dot.edge(node_id(state), node_id(c["to"]), label=label)

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
