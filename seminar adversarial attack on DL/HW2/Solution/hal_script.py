# Environment and HAL initialization
import sys, os
HAL_BASE = "../build/"
os.environ["HAL_BASE_PATH"] = HAL_BASE
sys.path.append(HAL_BASE+"lib/")
import hal_py
from hal_py import GateLibraryManager
hal_py.plugin_manager.load_all_plugins()
from hal_py import NetlistUtils
from hal_plugins.graph_algorithm import GraphAlgorithmPlugin
import itertools
graph_manager = hal_py.plugin_manager.get_plugin_instance('graph_algorithm')


#load netlist
netlist = hal_py.NetlistFactory.load_netlist("project2_cipher_v1.v", HAL_BASE + "share/hal/gate_libraries/NangateOpenCellLibrary.hgl")
#netlist = hal_py.NetlistFactory.load_netlist("project2_cipher_v2_obfuscated.v", HAL_BASE + "share/hal/gate_libraries/NangateOpenCellLibrary.hgl")

#find fsm
sccs = graph_manager.get_strongly_connected_components(netlist)
filtered_sccs = [scc for scc in sccs if len(scc) > 1]
target = min(filtered_sccs, key=len)

#create fsm module
module = netlist.create_module('combinational_and_sequential', netlist.get_top_module() ,target)

#create separate modules for combinationals and sequentials
combinational_gates = [gate for gate in module.get_gates() if gate.get_type().has_property(hal_py.GateTypeProperty.combinational)]
sequential_gates = [gate for gate in module.get_gates() if gate.get_type().has_property(hal_py.GateTypeProperty.sequential)]
combinational_module = netlist.create_module('combinational', module ,combinational_gates)
sequential_module = netlist.create_module('sequential', module ,sequential_gates)

#determine boolean functions
ffs = sequential_module.get_gates(lambda g : "FF" in g.get_type().get_name())

def dfs_from_net(net):
    gate_list = []
    stack = [net]
    while(stack):
        n = stack.pop()
        for endpoint in n.get_sources():
            gate = endpoint.get_gate()
            if gate not in gate_list:
                gate_list.append(gate)
                for net in gate.get_fan_in_nets():
                    if not net.is_global_input_net():
                        stack.append(net)
    return gate_list


bfs = []
states = []
negstates = []

for ff in ffs:
    #datapin = ff.get_type().get_pins(lambda g : g.get_type() == hal_py.PinType.data)[0]
    datapin = ff.get_type().get_pins(lambda g : g.get_type() == hal_py.PinType.none or g.get_type() == hal_py.PinType.data)[0]
    fanin_net = ff.get_fan_in_net(datapin)
    statepin = ff.get_type().get_pins(lambda g : g.get_type() == hal_py.PinType.state)[0]
    neg_statepin = ff.get_type().get_pins(lambda g : g.get_type() == hal_py.PinType.neg_state)[0]
    state_net = ff.get_fan_out_net(statepin)
    neg_state_net = ff.get_fan_out_net(neg_statepin)
    # fanin_func = hal_py.NetlistUtils.get_subgraph_function(fanin_net, combinational_module.get_gates())
    # bfs.append(fanin_func)
    states.append(state_net)
    negstates.append(neg_state_net)

    gts = dfs_from_net(fanin_net)
    for ff1 in ffs:
        gts.remove(ff1)
    fanin_func = hal_py.NetlistUtils.get_subgraph_function(fanin_net,gts)
    bfs.append(fanin_func)

print("The boolean functions are:")
for bf in bfs:
    print(bf)
    print()

vars = []
for bf in bfs:
    vars.extend(bf.get_variable_names())
vars = set(vars)


states = [f'net_{state.get_id()}' for state in states]
negstates = [f'net_{neg_state.get_id()}' for neg_state in negstates]
input = [variable for variable in vars if variable not in states and variable not in negstates]

print("The input signals are:")
for net_id in input:
    print(net_id)

#BFS Brute Force of FSM
unvisited = set(["0" * len(ffs)])
visited = set()
variables = {}
transitions = set()

while len(unvisited) > 0:
    current_state = unvisited.pop()
    visited.add(current_state)
    
    for idx, value in enumerate(current_state):
        value = int(value)
        q, negq = states[idx], negstates[idx]
        variables[q], variables[negq] = hal_py.BooleanFunction.Value(value), hal_py.BooleanFunction.Value(1 - value)

    for values in itertools.product([0, 1], repeat=len(input)):
        for net_id, net_value in zip(input, values):
            variables[net_id] = hal_py.BooleanFunction.Value(net_value)

        new_state = ""
        for i, ff in enumerate(ffs):
            result = bfs[i].evaluate(variables)
            new_state += str(result.value)

        transition_signal = ','.join(str(x) for x in values)
        transition = (current_state, new_state, transition_signal)
        transitions.add(transition)

        if new_state not in visited and new_state not in unvisited:
            unvisited.add(new_state)

print(transitions)

edges = {}
for src, dst, transition_signal in transitions:
    signals = transition_signal.split(',')
    observed_signals = edges.get((src, dst), [set() for _ in signals])
    for i, new_signal in enumerate(signals):
        observed_signals[i].add(new_signal)
    edges[(src, dst)] = observed_signals


def compute_signal(x):
    if len(x) > 1:
        return 'X'
    return x.pop()

processed_edges = set()
for (src, dst), observed_signals in edges.items():
    edge_transition_signals = ','.join(compute_signal(observed_signal) for observed_signal in observed_signals)
    processed_edges.add((src, dst, edge_transition_signals))

#Save FSM to .dot file
with open("fsm.dot", "w") as f:
    f.write("// FSM\n")
    f.write("digraph {\n")

    for src, dst, transition in processed_edges:
        f.write(f"\t{src} -> {dst} [label=\"{transition}\"]\n")
    f.write("}")

print('Finished successfully...')