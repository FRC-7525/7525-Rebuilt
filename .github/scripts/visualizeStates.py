import sys
import re
import networkx as nx
from pathlib import Path
import matplotlib.pyplot as plt

# States that do not need to have a trigger to idle
EXCLUDED_STATES = []

def get_states(manager_states_path):
    file = Path(manager_states_path).read_text(encoding='utf8')
    states = re.findall(r'\s*(?:\w+)\s*\((?:[\s\S]*?)\)[,;]', file, re.MULTILINE | re.IGNORECASE)
    return states

def parse_states(states):
    parsed_states = []
    for state in states:
        state_name_match = re.match(r'^\s*(\w+)\s*\(', state, re.IGNORECASE)
        sub_states = re.findall(r'\b(?:\w+\.)+\w+\b', state)
        parsed_states.append({
            'stateName': state_name_match[1] if state_name_match else None,
            'subStates': sub_states
        })
    return parsed_states

def get_triggers(manager_path):
    file = Path(manager_path).read_text(encoding='utf8')
    triggers = re.findall(r'addTrigger\(.+?\)', file, re.MULTILINE | re.IGNORECASE)
    return triggers

def parse_triggers(triggers):
    parsed_triggers = []
    for trigger in triggers:
        matched = re.search(r'\s*(\w+)\s*,\s*(\w+)\s*,\s*((?:[\w:]+)|(?:\(\)\s*->\s*[\w.]+\(\)))\s*\)', trigger)
        if matched:
            parsed_triggers.append({
                'from': matched[1],
                'to': matched[2],
                'condition': matched[3]
            })
    return parsed_triggers

def create_state_map(states, triggers):
    state_map = {state['stateName']: state for state in states}

    for trigger in triggers:
        state = state_map.get(trigger['from'])
        if state is not None:
            if 'connectionsTo' not in state:
                state['connectionsTo'] = []
            state['connectionsTo'].append({
                'to': trigger['to'],
                'condition': trigger['condition']
            })
    return state_map

def generate_graph(state_map):
    graph = nx.DiGraph()
    for (state_name, state_info) in state_map.items():
        graph.add_node(state_name, label=state_name)
        for connection in state_info.get('connectionsTo', []):
            graph.add_edge(state_name, connection['to'], weight=1)


    pos = nx.shell_layout(graph)
    pos["IDLE"] = (0, 0)

    fixed_nodes = ["IDLE"]
    pos = nx.spring_layout(graph, pos=pos, fixed=fixed_nodes, k=0.2)

    edge_labels = nx.get_edge_attributes(graph, 'label')

    nx.draw(graph, pos, with_labels=True, node_color='lightblue', node_size=1000, font_size=6, arrows=True)
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels, font_size=6)

    plt.title("State Machine Visualization")
    plt.tight_layout()
    plt.savefig("state_machine.png", format='png', dpi=300)


def checkConnectedToIdle(state_map):
    notToIdle = []
    for state_name, state in state_map.items():
        if state_name in EXCLUDED_STATES: continue
        for connections in state["connectionsTo"]:
            if connections.to == "IDLE": continue
            else: notToIdle.append(state_name)
    return notToIdle


def main(managerStatesPath, managerPath):
    states = get_states(managerStatesPath)
    states = parse_states(states)

    triggers = get_triggers(managerPath)
    triggers = parse_triggers(triggers)

    state_map = create_state_map(states, triggers)
    
    generate_graph(state_map)
    notToIdle = checkConnectedToIdle(state_map)
    if notToIdle:
        print(f"ERROR: The following states are not connected to IDLE: {', '.join(notToIdle)}")
        return 1
    else:
        print("Success: All states are connected to IDLE.")
    return 0
if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])