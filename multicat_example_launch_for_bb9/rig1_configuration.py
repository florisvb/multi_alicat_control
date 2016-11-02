ADDRESSES = ['A', 'B', 'C']
PRIMARY_FLOWRATE = 20
SSR_PORTS = [0,1]

def off(flowrate):
    bb9_addresses = ADDRESSES
    bb9_flowrates = [PRIMARY_FLOWRATE, PRIMARY_FLOWRATE, 0]
    ssr_ports = SSR_PORTS
    ssr_states = [0, 0]
    return bb9_addresses, bb9_flowrates, ssr_ports, ssr_states

def right(flowrate):
    bb9_addresses = ADDRESSES
    bb9_flowrates = [PRIMARY_FLOWRATE, PRIMARY_FLOWRATE-flowrate, flowrate]
    #bb9_flowrates = [PRIMARY_FLOWRATE, PRIMARY_FLOWRATE, flowrate]
    ssr_ports = SSR_PORTS
    ssr_states = [0, 1]
    return bb9_addresses, bb9_flowrates, ssr_ports, ssr_states
    
def left(flowrate):
    bb9_addresses = ADDRESSES
    bb9_flowrates = [PRIMARY_FLOWRATE-flowrate, PRIMARY_FLOWRATE, flowrate]
    #bb9_flowrates = [PRIMARY_FLOWRATE, PRIMARY_FLOWRATE, flowrate]
    ssr_ports = SSR_PORTS
    ssr_states = [1, 0]
    return bb9_addresses, bb9_flowrates, ssr_ports, ssr_states
    
    
'''

rostopic pub /rig1 multi_alicat_control/msg_action_and_flowrate '{action:  "right", flowrate: 2}'

'''
