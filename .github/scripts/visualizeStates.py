import sys
import re
from pathlib import Path
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

# ---------- Parsing ----------
def get_states(path):
    text = Path(path).read_text(encoding="utf8")
    return re.findall(r"\w+\s*\((?:[\s\S]*?)\)[,;]", text)

def parse_states(blocks):
    states = []
    for block in blocks:
        name = re.match(r"(\w+)\s*\(", block)
        if not name:
            continue
        subs = dict(
            (k.replace("States", ""), v)
            for k, v in re.findall(r"(\w+States)\.(\w+)", block)
        )
        states.append({
            "stateName": name.group(1),
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
                \s*(?:\w+\.)?(\w+)\s*,      # from
                \s*(?:\w+\.)?(\w+)\s*,      # to
                \s*(.+?)                    # condition
            \)\s*;
            """,
            t,
            re.VERBOSE | re.DOTALL,
        )
        if not m:
            continue
        cond = m.group(3).strip()
        cond = re.sub(r"^\(\)\s*->\s*", "", cond)
        cond = re.sub(r"(?:DRIVER|OPERATOR)_CONTROLLER::get", "", cond)
        cond = cond.replace("()", "")
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

# ---------- Visualization ----------
EDGE_COLORS = [
    "#60a5fa", "#4ade80", "#facc15",
    "#a78bfa", "#fb7185", "#22d3ee"
]

def draw_state_machine(state_map):
    G = nx.DiGraph()
    for s, info in state_map.items():
        G.add_node(s, **info)
    for s, info in state_map.items():
        for i, c in enumerate(info.get("connections", [])):
            G.add_edge(s, c["to"], condition=c["condition"], color=EDGE_COLORS[i % len(EDGE_COLORS)])

    # ---------- Layout ----------
    pos = nx.spring_layout(G, seed=42, k=3.5)  # larger k → more spacing
    # Scale layout to fill figure
    scale = 10
    pos = {k: (v[0]*scale, v[1]*scale) for k, v in pos.items()}

    fig, ax = plt.subplots(figsize=(18, 14))
    fig.patch.set_facecolor("#020617")
    ax.set_facecolor("#020617")
    plt.axis('off')

    # ---------- Draw edges first ----------
    for u, v, data in G.edges(data=True):
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        mid_x, mid_y = x2, y1
        # horizontal then vertical
        ax.plot([x1, mid_x], [y1, mid_y], color=data['color'], linewidth=1.5)
        ax.plot([mid_x, x2], [mid_y, y2], color=data['color'], linewidth=1.5)
        ax.annotate("",
                    xy=(x2, y2),
                    xytext=(mid_x, mid_y),
                    arrowprops=dict(arrowstyle="-|>", color=data['color'], lw=1.5))
        label_x = (x1 + mid_x)/2
        label_y = y1 + 0.03
        ax.text(label_x, label_y, data['condition'],
                fontsize=8, color=data['color'], ha='center', va='bottom',
                bbox=dict(boxstyle="round,pad=0.2", facecolor="#020617", edgecolor=data['color']))

    # ---------- Draw nodes on top ----------
    for node, info in G.nodes(data=True):
        x, y = pos[node]
        lines = [
            node,
            f"Intake: {info['Intake']}",
            f"Hopper: {info['Hopper']}",
            f"Shooter: {info['Shooter']}",
            f"Climber: {info['Climber']}"
        ]
        max_line_len = max(len(line) for line in lines)
        width = max_line_len * 0.02 + 0.15
        height = len(lines) * 0.05 + 0.1

        bbox = FancyBboxPatch((x - width/2, y - height/2), width, height,
                              boxstyle="round,pad=0.02",
                              facecolor="#1e293b",
                              edgecolor="#64748b",
                              linewidth=1.5)
        ax.add_patch(bbox)
        ax.text(x, y, "\n".join(lines), fontsize=10, ha='center', va='center', color="white")

    # ---------- Expand plot limits ----------
    all_x = [v[0] for v in pos.values()]
    all_y = [v[1] for v in pos.values()]
    ax.set_xlim(min(all_x)-1, max(all_x)+1)
    ax.set_ylim(min(all_y)-1, max(all_y)+1)

    plt.tight_layout()
    plt.savefig("state_machine.png", dpi=300)
    plt.show()
    print("✅ state_machine.png generated")

# ---------- Main ----------
def main(states_path, triggers_path):
    states = parse_states(get_states(states_path))
    triggers = parse_triggers(get_triggers(states_path))
    draw_state_machine(create_state_map(states, triggers))

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
