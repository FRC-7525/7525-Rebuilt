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
        # Remove lambda wrapper
        cond = re.sub(r"^\(\)\s*->\s*", "", cond)
        # Remove controller prefixes
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

# ---------- Graph ----------

EDGE_COLORS = [
    "#60a5fa", "#4ade80", "#facc15",
    "#a78bfa", "#fb7185", "#22d3ee"
]

def nid(name):
    return name.replace("-", "_")

def edge_label_box(cond_text, color):
    """Return HTML table label for edge with background same as graph"""
    return f"""<
    <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="2" BGCOLOR="#020617">
        <TR><TD><FONT COLOR="{color}" POINT-SIZE="8">{cond_text}</FONT></TD></TR>
    </TABLE>
    >"""

def generate_graph(state_map):
    dot = Digraph("StateMachine", engine="dot", format="png")
    dot.attr(
        bgcolor="#020617",
        rankdir="TB",
        splines="ortho",   # strictly vertical+horizontal edges
        nodesep="0.8",
        ranksep="1.0",
        fontname="Helvetica"
    )

    # ---------- Nodes ----------
    for state, info in state_map.items():
        label = (
            f"{state}\n\n"
            f"Intake: {info['Intake']}\n"
            f"Hopper: {info['Hopper']}\n"
            f"Shooter: {info['Shooter']}\n"
            f"Climber: {info['Climber']}"
        )
        dot.node(
            nid(state),
            label=label,
            shape="box",
            style="rounded,filled",
            fillcolor="#1e293b",
            color="#64748b",
            fontcolor="white"
        )

    # ---------- Edges ----------
    color_i = 0
    for state, info in state_map.items():
        for c in info.get("connections", []):
            color = EDGE_COLORS[color_i % len(EDGE_COLORS)]
            color_i += 1

            dot.edge(
                nid(state),
                nid(c["to"]),
                label=edge_label_box(c["condition"], color),
                color=color,
                fontcolor=color,
                penwidth="1.4"
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
