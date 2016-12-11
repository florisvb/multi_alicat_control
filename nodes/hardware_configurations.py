#!/usr/bin/env python
from __future__ import division
from optparse import OptionParser
import roslib
import rospy
import rosparam
import numpy as np
from std_msgs.msg import Float32, Header, String
from multi_alicat_control.msg import msg_bb9, msg_phidget_interface_ssr, msg_action_and_flowrate
import time
import imp

class Rig(object):
    def __init__(self,  action_and_flowrate_topic='/rig1', 
                        bb9_topic='/alicat_bb9', 
                        ssr_topic='/phidgets_interface_ssr', 
                        rig_function_configuration='path_to_py_file',
                        ssr_off_delay=5):
        nodename = action_and_flowrate_topic.split('/')[1]
        rospy.init_node(nodename)
        self.ssr_off_delay = ssr_off_delay # time, seconds, to wait between turning off flow and ssr on an off action
        
        self.subscriber = rospy.Subscriber(action_and_flowrate_topic, msg_action_and_flowrate, self.control_hardware, queue_size=10)
        self.bb9_publisher = rospy.Publisher(bb9_topic, msg_bb9, queue_size=10)
        self.ssr_publisher = rospy.Publisher(ssr_topic, msg_phidget_interface_ssr, queue_size=10)
        
        self.rig_functions = imp.load_source('rig_functions', rig_function_configuration)
        
        rospy.spin()

    def control_hardware(self, action_and_flowrate):
        bb9_addresses, bb9_flowrates, ssr_ports, ssr_states = self.rig_functions.__getattribute__(action_and_flowrate.action)(action_and_flowrate.flowrate)

        tnow = rospy.Time.now()
        
        bb9 = msg_bb9()
        bb9.header.stamp = tnow
        bb9.addresses = bb9_addresses
        bb9.flowrates = bb9_flowrates
        
        ssr = msg_phidget_interface_ssr()
        ssr.header.stamp = tnow
        
        if ssr_ports is not None:
            ssr.ports = ssr_ports
            ssr.states = ssr_states


        if action_and_flowrate.action == 'off':
            self.bb9_publisher.publish(bb9)
            time.sleep(self.ssr_off_delay)
            self.ssr_publisher.publish(ssr)
        else:
            self.ssr_publisher.publish(ssr)
            #time.sleep(self.ssr_off_delay)
            self.bb9_publisher.publish(bb9)
        
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--action_and_flowrate_topic", type="str", dest="action_and_flowrate_topic", default='/rig1',
                        help="action_and_flowrate_topic")
    parser.add_option("--bb9_topic", type="str", dest="bb9_topic", default="/alicat_bb9",
                        help="topic to publish bb9 messages, for controlling alicats through the bb9 node (one of these topics per bb9 device)")
    parser.add_option("--ssr_topic", type="str", dest="ssr_topic", default="/phidgets_interface_ssr",
                        help="topic to publish ssr messages, for controlling through phidgets interface kit (one of these topics per interface kit)")        
    parser.add_option("--rig_function_configuration", type="str", dest="rig_function_configuration", default='none',
                        help="full path to configuration py file listing the rig functions")

    (options, args) = parser.parse_args()
    
    
    rig = Rig(options.action_and_flowrate_topic, options.bb9_topic, options.ssr_topic, options.rig_function_configuration)
