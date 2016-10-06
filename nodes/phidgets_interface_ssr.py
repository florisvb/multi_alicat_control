#!/usr/bin/env python
from __future__ import division

# ROS imports
import roslib
import rospy
import rosparam
from multi_alicat_control.msg import msg_phidget_interface_ssr

#Basic imports
from ctypes import *
import time
import numpy as np
import sys
import random

#Phidget specific imports
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, ErrorEventArgs, InputChangeEventArgs, OutputChangeEventArgs, SensorChangeEventArgs
from Phidgets.Devices.InterfaceKit import InterfaceKit

###############################################################################
###############################################################################
class PhidgetsDAQ:

    def __init__(self):
        rospy.init_node('phidgets_interface_ssr', anonymous=True)
        self.publish_rate = 1
        
    def initialize_digital_outputs(self, phidget):
        '''
        subscribes to /phidgets_interface_ssr topic, and connects received messages with setting the output on the phidget
        '''
        self.phidget = phidget
        self.subscriber = rospy.Subscriber('/phidgets_interface_ssr', msg_phidget_interface_ssr, self.set_digital_output)
        self.publisher  = rospy.Publisher('/phidgets_interface_ssr/output_data', msg_phidget_interface_ssr, queue_size=10)
        self.pins = [0, 1, 2, 3]
        
        self.msg_prototype = msg_phidget_interface_ssr()
            
            
    def set_digital_output(self, digital_output):
        for i in range(len(digital_output.ports)):
            self.phidget.setOutputState(digital_output.ports[i], digital_output.states[i])    
            if i not in self.pins:
                self.pins.append(i)
        self.pins.sort()
        
    def run(self):
        rate = rospy.Rate(self.publish_rate) # 10hz
        while not rospy.is_shutdown():
            if len(self.pins) > 0:
                data = [int(self.phidget.getOutputState(pin)) for pin in self.pins]
                self.msg_prototype.ports = self.pins
                self.msg_prototype.states = data
                self.publisher.publish(self.msg_prototype)
                rate.sleep()
        
def connect_to_phidget(SensorChangedFunction, serial_number=None):
    #Create an interfacekit object
    try:
        interfaceKit = InterfaceKit()
    except RuntimeError as e:
        print("Runtime Exception: %s" % e.details)
        print("Exiting....")
        exit(1)

    #Information Display Function
    def displayDeviceInfo():
        print("|------------|----------------------------------|--------------|------------|")
        print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
        print("|------------|----------------------------------|--------------|------------|")
        print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (interfaceKit.isAttached(), interfaceKit.getDeviceName(), interfaceKit.getSerialNum(), interfaceKit.getDeviceVersion()))
        print("|------------|----------------------------------|--------------|------------|")
        print("Number of Digital Inputs: %i" % (interfaceKit.getInputCount()))
        print("Number of Digital Outputs: %i" % (interfaceKit.getOutputCount()))
        print("Number of Sensor Inputs: %i" % (interfaceKit.getSensorCount()))

    #Event Handler Callback Functions
    def interfaceKitAttached(e):
        attached = e.device
        print("InterfaceKit %i Attached!" % (attached.getSerialNum()))

    def interfaceKitDetached(e):
        detached = e.device
        print("InterfaceKit %i Detached!" % (detached.getSerialNum()))

    def interfaceKitError(e):
        try:
            source = e.device
            print("InterfaceKit %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))

    #Main Program Code
    try:
        interfaceKit.setOnAttachHandler(interfaceKitAttached)
        interfaceKit.setOnDetachHandler(interfaceKitDetached)
        interfaceKit.setOnErrorhandler(interfaceKitError)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)

    print("Opening phidget object....")

    try:
        if serial_number is not None:
            interfaceKit.openPhidget(serial_number)
        else:
            interfaceKit.openPhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)

    print("Waiting for attach....")

    try:
        interfaceKit.waitForAttach(10000)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        try:
            interfaceKit.closePhidget()
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Exiting....")
            exit(1)
        print("Exiting....")
        exit(1)
    else:
        displayDeviceInfo()

    phidget = interfaceKit
    
    return phidget

if __name__ == '__main__':
    
    print 'connecting to phidget'
    phidgets_daq = PhidgetsDAQ()
    serial_number = rospy.get_param('/phidgets_interface_ssr/serial_number', None)
    phidget = connect_to_phidget(None, serial_number=serial_number)
    
    phidgets_daq.initialize_digital_outputs(phidget)
    phidgets_daq.run()

'''

rostopic pub /phidgets_interface_ssr multi_alicat_control/msg_phidget_interface_ssr '{ports:  [0, 1], states: [0, 1]}'

'''
