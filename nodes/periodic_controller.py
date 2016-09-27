#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
import numpy as np
from std_msgs.msg import Float32, Header, String
from multi_alicat_control.msg import msg_action_and_flowrate
import time
import imp

class PeriodicFlowController(object):
    def __init__(self,  action_and_flowrate_topic='/rig1', 
                        delay_before_first_pulse=10,
                        pulse_length=10,
                        inter_pulse_length=10,
                        total_experiment_length=7200,
                        flowrates=[1,5],
                        randomize_flowrate_order=True,
                        off_action='off',
                        on_actions=['left', 'right'],
                        randomize_on_actions_order=False):
        
        self.delay_before_first_pulse = delay_before_first_pulse
        self.pulse_length = pulse_length
        self.inter_pulse_length = inter_pulse_length
        self.total_experiment_length = total_experiment_length
        self.off_action = off_action
         
        indices = range(len(flowrates)) 
        if randomize_flowrate_order:
            np.random.shuffle(indices)
        self.flowrates = [flowrates[i] for i in indices]
            
        indices = range(len(on_actions))
        if randomize_on_actions_order:
            np.random.shuffle(indices)
        self.on_actions = [on_actions[i] for i in indices]
        
        nodename = action_and_flowrate_topic.split('/')[1] + '_' + 'periodic_controller'
        rospy.init_node(nodename)
        self.publisher = rospy.Publisher(action_and_flowrate_topic, msg_action_and_flowrate, queue_size=10)
        
        time.sleep(2) # allow other stuff to start
        self.main()
        
    def get_current_flowrate(self):
        try:
            flowrate = self.iterator_flowrate.next()
        except StopIteration:
            self.iterator_flowrate = iter(self.flowrates)
            flowrate = self.iterator_flowrate.next()
        except AttributeError:
            self.iterator_flowrate = iter(self.flowrates)
            flowrate = self.iterator_flowrate.next()
        return flowrate
        
    def get_current_on_action(self):
        try:
            on_action = self.iterator_on_action.next()
        except StopIteration:
            self.iterator_on_action = iter(self.on_actions)
            on_action = self.iterator_on_action.next()
        except AttributeError:
            self.iterator_on_action = iter(self.on_actions)
            on_action = self.iterator_on_action.next()
        return on_action


    def main(self):
        msg = msg_action_and_flowrate()
        msg.header.stamp = rospy.Time.now()
        msg.action = self.off_action
        msg.flowrate = 0
        self.publisher.publish(msg)
        self.time_started = time.time()
        time.sleep(self.delay_before_first_pulse)
        while not rospy.is_shutdown():
            # ON ACTION
            flowrate = self.get_current_flowrate()
            on_action = self.get_current_on_action()
            msg = msg_action_and_flowrate()
            msg.header.stamp = rospy.Time.now()
            msg.action = on_action
            msg.flowrate = flowrate
            self.publisher.publish(msg)
            time.sleep(self.pulse_length)
            
            # OFF ACTION
            msg = msg_action_and_flowrate()
            msg.header.stamp = rospy.Time.now()
            msg.action = self.off_action
            msg.flowrate = 0
            self.publisher.publish(msg)
            time.sleep(self.inter_pulse_length)
            
            if time.time() - self.time_started > self.total_experiment_length:
                break
        
        
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--action_and_flowrate_topic", type="str", dest="action_and_flowrate_topic", default='/rig1',
                        help="action_and_flowrate_topic")
    parser.add_option("--delay_before_first_pulse", type="float", dest="delay_before_first_pulse", default=10,
                        help="seconds to wait before first on pulse")
    parser.add_option("--pulse_length", type="float", dest="pulse_length", default=10,
                        help="pulse length, seconds")
    parser.add_option("--inter_pulse_length", type="float", dest="inter_pulse_length", default=10,
                        help="time between pulses, seconds")
    parser.add_option("--total_experiment_length", type="float", dest="total_experiment_length", default=7200,
                        help="how long is the experiment, seconds")
    parser.add_option("--flowrates", type="str", dest="flowrates", default="[1, 0.2]",
                        help="list of flow rates to use")
    parser.add_option("--on_actions", type="str", dest="on_actions", default="['left', 'right']",
                        help="list of on actions - must correspond to rig configuration functions")        
    parser.add_option("--randomize_flowrate_order", type="int", dest="randomize_flowrate_order", default=True,
                        help="randomize the order of the flow rates")
    parser.add_option("--off_action", type="str", dest="off_action", default="off",
                        help="name of off action, if it is literally off, be sure to use double quotes")
    parser.add_option("--randomize_on_actions_order", type="int", dest="randomize_on_actions_order", default=False,
                        help="randomize the order of the on actions")
    parser.add_option("--periodic_controller_configuration", type="str", dest="periodic_controller_configuration", default='',
                        help="full path to a py configuration file for the periodic controller")

    (options, args) = parser.parse_args()
    
    if len(options.periodic_controller_configuration) > 0:
        periodic_controller_configuration = imp.load_source('periodic_controller_configuration', options.periodic_controller_configuration)
        options = periodic_controller_configuration
    
    periodic_flow_controller = PeriodicFlowController(  action_and_flowrate_topic=options.action_and_flowrate_topic, 
                                                        delay_before_first_pulse=options.delay_before_first_pulse,
                                                        pulse_length=options.pulse_length,
                                                        inter_pulse_length=options.inter_pulse_length,
                                                        total_experiment_length=options.total_experiment_length,
                                                        flowrates=eval(options.flowrates),
                                                        randomize_flowrate_order=options.randomize_flowrate_order,
                                                        off_action=options.off_action,
                                                        on_actions=eval(options.on_actions),
                                                        randomize_on_actions_order=options.randomize_on_actions_order,
                                                        )
                                                        
                                                        
