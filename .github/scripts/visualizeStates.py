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

# ---------- Graphviz ----------

EDGE_COLORS = [
    "#60a5fa", "#4ade80", "#facc15",
    "#a78bfa", "#fb7185", "#22d3ee"
]

NODE_COLORS = [
    "#1f2937", "#334155", "#475569", "#64748b", "#7f9cf5", "#a78bfa"
]

def generate_graph(state_map):
    dot = Digraph("StateMachine", format="png", engine="dot")
    dot.attr(
        bgcolor="#020617",
        rankdir="TB",
        splines="ortho",
        nodesep="1.2",
        ranksep="1.5",
        fontname="Helvetica"
    )

    # ---------- Nodes ----------
    for i, (state, info) in enumerate(state_map.items()):
        label = f"{state}\nIntake: {info['Intake']}\nHopper: {info['Hopper']}\nShooter: {info['Shooter']}\nClimber: {info['Climber']}"
        dot.node(
            state,
            label=label,
            shape="box",
            style="rounded,filled",
            fillcolor=NODE_COLORS[i % len(NODE_COLORS)],
            fontcolor="white",
            color="#ffffff",
        )

    # ---------- Edges ----------
    color_i = 0
    for state, info in state_map.items():
        for c in info.get("connections", []):
            color = EDGE_COLORS[color_i % len(EDGE_COLORS)]
            color_i += 1
            dot.edge(
                state,
                c["to"],
                label=c["condition"],
                fontcolor=color,
                fontsize="10",
                color=color,
                arrowsize="1.2",
                penwidth="1.5",
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
