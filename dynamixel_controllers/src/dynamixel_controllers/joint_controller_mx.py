# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__author__ = 'Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'

import math
import time

import rospy

from dynamixel_driver.dynamixel_const import *

from dynamixel_controllers.srv import SetSpeed
from dynamixel_controllers.srv import SetPosition
from dynamixel_controllers.srv import SetGoalTorque
from dynamixel_controllers.srv import SetAngleLimits
from dynamixel_controllers.srv import TorqueEnable
from dynamixel_controllers.srv import SetTorqueLimit
from dynamixel_controllers.srv import TorqueControlModeEnable
from dynamixel_controllers.srv import SetPID

from std_msgs.msg import Float64
from std_msgs.msg import UInt8
from std_msgs.msg import Bool

from dynamixel_msgs.msg import MotorStateList
from dynamixel_msgs.msg import JointState

class JointControllerMX:
    def __init__(self, dxl_io, controller_namespace, port_namespace):

        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        self.joint_name = rospy.get_param(self.controller_namespace + '/joint_name')
        self.joint_speed = rospy.get_param(self.controller_namespace + '/joint_speed', 1.0)
        self.torque_limit = rospy.get_param(self.controller_namespace + '/joint_torque_limit', None)
        self.wheel_mode = False
        self.torque_control_mode = False
       
        # create services for setting dynamixel registers
        self.speed_service = rospy.Service(self.controller_namespace + '/set_speed', SetSpeed, self.process_set_speed)
        self.torque_enable_service = rospy.Service(self.controller_namespace + '/torque_enable', TorqueEnable, self.process_torque_enable)
        self.PID_service = rospy.Service(self.controller_namespace + '/set_PID', SetPID, self.process_set_PID)
        self.angle_limits_service = rospy.Service(self.controller_namespace + '/set_angle_limits', SetAngleLimits, self.process_set_angle_limits)
        self.torque_limit_service = rospy.Service(self.controller_namespace + '/set_torque_limit', SetTorqueLimit, self.process_set_torque_limit)
        self.position_service = rospy.Service(self.controller_namespace + '/set_position', SetPosition, self.process_set_position)
        self.torque_control_mode_enable_service = rospy.Service(self.controller_namespace + '/torque_control_mode_enable', TorqueControlModeEnable, self.process_torque_control_mode_enable)
        self.goal_torque_service = rospy.Service(self.controller_namespace + '/set_goal_torque', SetGoalTorque, self.process_set_goal_torque)

        # set motor parameters from config file
        self.motor_id = rospy.get_param(self.controller_namespace + '/motor/id')
        # these min and max angles are unchangeable limits for this joint
        self.min_cw_limit = rospy.get_param(self.controller_namespace + '/motor/cw_limit')
        self.max_ccw_limit = rospy.get_param(self.controller_namespace + '/motor/ccw_limit')
        # these are the changeable angle limits
        self.cw_limit = self.min_cw_limit
        self.ccw_limit = self.max_ccw_limit
        
        self.joint_state = JointState(name=self.joint_name, motor_ids=[self.motor_id])

    def initialize(self):
        # verify that the expected motor is connected and responding
        available_ids = rospy.get_param('dynamixel/%s/connected_ids' % self.port_namespace, [])
        if not self.motor_id in available_ids:
            rospy.logwarn('The specified motor id is not connected and responding.')
            rospy.logwarn('Available ids: %s' % str(available_ids))
            rospy.logwarn('Specified id: %d' % self.motor_id)
            return False
        
        self.MODEL_NUMBER = rospy.get_param('dynamixel/%s/%d/model_number' % (self.port_namespace, self.motor_id))    
        self.RADIANS_PER_ENCODER_TICK = rospy.get_param('dynamixel/%s/%d/radians_per_encoder_tick' % (self.port_namespace, self.motor_id))
        self.ENCODER_TICKS_PER_RADIAN = rospy.get_param('dynamixel/%s/%d/encoder_ticks_per_radian' % (self.port_namespace, self.motor_id))
        self.ENCODER_RESOLUTION = rospy.get_param('dynamixel/%s/%d/encoder_resolution' % (self.port_namespace, self.motor_id))
        self.MAX_POSITION = self.ENCODER_RESOLUTION - 1
        self.VELOCITY_PER_TICK = rospy.get_param('dynamixel/%s/%d/radians_second_per_encoder_tick' % (self.port_namespace, self.motor_id))
        self.MAX_VELOCITY = rospy.get_param('dynamixel/%s/%d/max_velocity' % (self.port_namespace, self.motor_id))
        self.MIN_VELOCITY = self.VELOCITY_PER_TICK
        self.STALL_AMPS_PER_LOAD_TICK = rospy.get_param('dynamixel/%s/%d/stall_amps_per_load_tick' % (self.port_namespace, self.motor_id))        
        self.STALL_CURRENT = rospy.get_param('dynamixel/%s/%d/stall_current' % (self.port_namespace, self.motor_id))        
                        
        #push config parameters into the actual dynamixel registers
        self.set_angle_limits(self.min_cw_limit, self.max_ccw_limit)
        if self.torque_limit is not None: self.set_torque_limit(self.torque_limit)
        self.set_speed(self.joint_speed)
        if self.MODEL_NUMBER in DXL_MX_MODEL_NUMBERS:
            self.set_torque_control_mode_enable(False)
        self.set_torque_enable(True) #turn on the dynamixel torque!        
                 
        return True

    def start(self):
        self.running = True
        self.joint_state_pub = rospy.Publisher(self.controller_namespace + '/state', JointState)
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', Float64, self.process_command)
        self.motor_states_sub = rospy.Subscriber('motor_states/%s' % self.port_namespace, MotorStateList, self.process_motor_states)
        self.dxl_io.enable_feedback(self.motor_id)
        
    def stop(self):
        self.running = False
        self.joint_state_pub.unregister()
        self.motor_states_sub.unregister()
        self.command_sub.unregister()
        self.speed_service.shutdown('normal shutdown')
        self.torque_enable_service.shutdown('normal shutdown')
        self.PID_service.shutdown('normal shutdown')
        self.angle_limits_service.shutdown('normal shutdown')
        self.torque_limit_service.shutdown('normal shutdown')
        self.position_service.shutdown('normal shutdown')
        self.torque_control_mode_enable_service.shutdown('normal shutdown')
        self.goal_torque_service.shutdown('normal shutdown')

    def set_torque_enable(self, torque_enable):
        rospy.loginfo("%s set torque enable = %s" %(self.controller_namespace, str(torque_enable)))
        self.dxl_io.set_torque_enabled(self.motor_id, torque_enable)

    def set_torque_control_mode_enable(self, torque_control_mode_enable):
        rospy.loginfo("%s set torque control mode = %s" %(self.controller_namespace, str(torque_control_mode_enable)))
        self.torque_control_mode = torque_control_mode_enable
        self.dxl_io.set_torque_control_mode_enabled(self.motor_id, torque_control_mode_enable)

    def set_speed(self, speed):
        if self.wheel_mode == True:
            raw_speed =  int(round(speed / self.VELOCITY_PER_TICK))
        else:
            raw_speed = max(1, int(round(speed / self.VELOCITY_PER_TICK)))
         
        rospy.loginfo("%s setting speed= %f" %(self.controller_namespace, speed))
        self.dxl_io.set_speed(self.motor_id, raw_speed)
 
    def set_position(self, position):
        if position < self.cw_limit: position = self.cw_limit
        elif position > self.ccw_limit: position = self.ccw_limit
        rospy.loginfo("%s setting position= %f" %(self.controller_namespace, position))
        self.dxl_io.set_position(self.motor_id, self.pos_rad_to_raw(position))
            
    def set_goal_torque(self, goal_torque): #goal torque measured in amps
        self.dxl_io.set_goal_torque(self.motor_id, self.goal_torque_to_raw(goal_torque))
        
    def set_torque_limit(self, new_torque_limit): #torque limit from 0 to 1
        #rospy.loginfo("new_torque_limit: %f " % new_torque_limit)
        if new_torque_limit > 1: new_torque_limit = 1.0         
        elif new_torque_limit < 0: new_torque_limit = 0
        self.torque_limit = new_torque_limit 
        torque_limit_raw = int(DXL_MAX_TORQUE_TICK * new_torque_limit)
        rospy.loginfo("{0} setting torque limit: {1:f} ".format(self.controller_namespace, new_torque_limit))
        self.dxl_io.set_torque_limit(self.motor_id, torque_limit_raw)
        
    def set_angle_limits(self, cw_limit_new, ccw_limit_new):
        #this method sets the angle limits, and as per dynamixel operation
        #setting these limits both to zero puts the dynamixel in wheel mode
        if cw_limit_new < self.min_cw_limit: cw_limit_new = self.min_cw_limit
        if ccw_limit_new > self.max_ccw_limit: ccw_limit_new = self.max_ccw_limit
        self.cw_limit = cw_limit_new
        self.ccw_limit = ccw_limit_new
        cw_limit_raw = self.pos_rad_to_raw(cw_limit_new)
        ccw_limit_raw = self.pos_rad_to_raw(ccw_limit_new)
        rospy.loginfo("%s setting limits: %f %f " %(self.controller_namespace, cw_limit_new, ccw_limit_new))
        entering_position_mode = False
        if (cw_limit_raw == 0) and (ccw_limit_raw == 0):
            self.wheel_mode = True
        else:
            if self.wheel_mode: entering_position_mode = True
            #changing from wheel mode to position mode
            self.wheel_mode = False
        
        #dynamixels are terrible, and if transitioning from wheel mode to
        #position mode, they go unresponsive for .1 seconds or so, then return with bad packets
        if entering_position_mode: self.dxl_io.disable_feedback(self.motor_id)
        self.dxl_io.set_angle_limits(self.motor_id, cw_limit_raw, ccw_limit_raw )        
        if entering_position_mode:
            time.sleep(.2)
            self.dxl_io.enable_feedback(self.motor_id)
        
                   
    def set_PID(self, p_gain, i_gain, d_gain):
        self.dxl_io.set_p_gain(self.motor_id, p_gain)
        self.dxl_io.set_i_gain(self.motor_id, i_gain)
        self.dxl_io.set_d_gain(self.motor_id, d_gain)
    
    def process_set_speed(self, req):
        self.set_speed(req.speed)
        return [] # success
    
    def process_set_position(self, req):
        self.set_position(req.position)
        return []

    def process_set_goal_torque(self, req):
        self.set_goal_torque(req.goal_torque)
        return []
    
    def process_torque_enable(self, req):
        self.set_torque_enable(req.torque_enable)
        return []

    def process_torque_control_mode_enable(self, req):
        self.set_torque_control_mode_enable(req.torque_control_mode_enable)
        return []

    def process_set_torque_limit(self, req):
        self.set_torque_limit(req.torque_limit)
        return []

    def process_set_angle_limits(self, req):
        self.set_angle_limits(req.cw_limit, req.ccw_limit)
        return []
 
    def process_set_PID(self, req):
        self.set_PID(req.p_gain, req.i_gain, req.d_gain)
        return []
    
    def process_motor_states(self, state_list):
        if self.running:
            state = filter(lambda state: state.id == self.motor_id, state_list.motor_states)
            if state:
                state = state[0]
                self.joint_state.motor_temps = [state.temperature]
                self.joint_state.goal_pos = self.raw_to_rad(state.goal)
                self.joint_state.current_pos = self.raw_to_rad(state.position)
                self.joint_state.error = self.raw_to_rad(state.error)
                self.joint_state.velocity = (state.speed / float(DXL_MAX_SPEED_TICK)) * self.MAX_VELOCITY
                self.joint_state.load = state.load
                self.joint_state.current = state.current
                self.joint_state.is_moving = state.moving
                self.joint_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                
                self.joint_state_pub.publish(self.joint_state)

    def process_command(self, msg):
        raise NotImplementedError

    def raw_to_rad(self, raw):
        return (raw * self.RADIANS_PER_ENCODER_TICK)

    def pos_rad_to_raw(self, pos_rad):
        return int(pos_rad * self.ENCODER_TICKS_PER_RADIAN)
    
    def goal_torque_to_raw(self, goal_torque):
        #goal torque is measured in amps
        return int(goal_torque / DXL_CURRENT_PER_TICK)

    