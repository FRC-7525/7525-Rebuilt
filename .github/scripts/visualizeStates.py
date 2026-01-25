import sys
import re
import networkx as nx
from pathlib import Path
import matplotlib.pyplot as plt

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
        m = re.search(
            r"\s*(\w+)\s*,\s*(\w+)\s*,\s*((?:[\w:]+)|(?:\(\)\s*->\s*[\w.]+\(\)))\s*\)",
            t,
        )
        if m:
            parsed.append({"from": m[1], "to": m[2], "condition": m[3]})
    return parsed

def create_state_map(states, triggers):
    state_map = {s["stateName"]: s for s in states}
    for t in triggers:
        state = state_map.get(t["from"])
        if state is not None:
            state.setdefault("connectionsTo", []).append(t)
    return state_map

# ---------- Visualization ----------
def generate_graph(state_map):
    G = nx.DiGraph()

    for state, info in state_map.items():
        G.add_node(state)
        for c in info.get("connectionsTo", []):
            G.add_edge(state, c["to"], label=c["condition"])

    # Layout
    pos = nx.spring_layout(G, k=0.9, seed=42)
    if "IDLE" in pos:
        pos["IDLE"] = (0, 0)

    # Styling
    plt.figure(figsize=(18, 18))
    ax = plt.gca()
    ax.set_facecolor("#f8f9fb")

    idle_nodes = ["IDLE"] if "IDLE" in G else []
    other_nodes = [n for n in G.nodes if n != "IDLE"]

    nx.draw_networkx_nodes(
        G,
        pos,
        nodelist=other_nodes,
        node_size=2600,
        node_color="#cfe8ff",
        edgecolors="#2b6cb0",
        linewidths=1.5,
    )

    nx.draw_networkx_nodes(
        G,
        pos,
        nodelist=idle_nodes,
        node_size=3400,
        node_color="#ffe8cc",
        edgecolors="#c05621",
        linewidths=2.5,
    )

    nx.draw_networkx_edges(
        G,
        pos,
        arrows=True,
        arrowstyle="-|>",
        arrowsize=18,
        width=1.4,
        edge_color="#4a5568",
        connectionstyle="arc3,rad=0.08",
    )

    nx.draw_networkx_labels(
        G,
        pos,
        font_size=10,
        font_weight="bold",
        font_family="DejaVu Sans",
    )

    edge_labels = nx.get_edge_attributes(G, "label")
    nx.draw_networkx_edge_labels(
        G,
        pos,
        edge_labels=edge_labels,
        font_size=8,
        rotate=False,
        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.85),
    )

    plt.title("State Machine", fontsize=20, fontweight="bold", pad=20)
    plt.axis("off")
    plt.tight_layout()
    plt.savefig("state_machine.png", dpi=300)
    plt.show()

# ---------- Main ----------
def main(managerStatesPath, managerPath):
    states = parse_states(get_states(managerStatesPath))
    triggers = parse_triggers(get_triggers(managerPath))
    state_map = create_state_map(states, triggers)
    generate_graph(state_map)
    print("Success!")

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
