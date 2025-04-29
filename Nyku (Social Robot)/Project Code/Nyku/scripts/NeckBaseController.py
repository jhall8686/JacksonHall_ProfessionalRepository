#!/usr/bin/env python

import rospy
from nyku.msg import *
from dynamixel_sdk_examples.srv import *
from AX_12 import Ax12
from std_msgs.msg import UInt8, Empty
import yaml

class NykuMotorController:
    def __init__(self,config_file, port):
        
        with open(config_file,'r') as file:
            config = yaml.load(file, yaml.Loader)
        Ax12.DEVICENAME = port
        Ax12.connect()
        Ax12.open_port()
        Ax12.set_baudrate()
        self.motors = []
        self.position_dict = dict()
        self.current_positions = dict()
        self.updated = [False for i in range(6)]
        for motor_name, info in config.items():
            self.position_dict[info['ID']] = []
            ax = Ax12(info['ID'])
            ax.set_ccw_angle_limit(info['CCW_Angle_Limit'])
            ax.set_cw_angle_limit(info['CW_Angle_Limit'])
            ax.set_moving_speed(info['Default_Moving_Speed'])
            self.motors.append(ax)

    def add_new_position(self,data):
        self.position_dict[data.id].append((data.position,None))

    def add_new_vel_pos(self,data):
        self.position_dict[data.id].append((data.position,data.velocity))

    def bulk_new_position(self,data):
        for item in data.controls: 
            id = item.id
            position = item.position   
            self.position_dict[id].append((position, None))

    def bulk_new_vel_pos(self,data):
        for item in data.data:
            id = item.id
            position = item.position
            velocity = item.velocity
            self.position_dict[id].append((position,velocity))
    
    def bulk_new_time_pos(self,data):
        for item in data.controls:
            id = item.id
            position = item.position
            time = item.time
            if not self.position_dict[id]:
                velocity = abs(position - self.current_positions[id]) // (2 * time)
                print("Position %d - Current Position %d / 2 * Time %d = Velocity %d" %(position, self.current_positions[id], time, velocity))
            else:
                velocity = abs(position - self.position_dict[id][-1][0]) // (2 * time)
            self.position_dict[id].append((position,velocity))

    def ros_init(self):
        rospy.init_node('nyku_motor_controller_node')

        rospy.Subscriber('neck_base/add_next_position', SetPosition, self.add_new_position)
        rospy.Subscriber('neck_base/add_next_velpos', SetVelPos, self.add_new_vel_pos)
        rospy.Subscriber('neck_base/clear_motor_queue',UInt8,self.clear_motor)
        rospy.Subscriber('neck_base/clear_queue',Empty,self.clear_all)
        rospy.Subscriber('neck_base/add_bulk_position',SetBulkPosition,self.bulk_new_position)
        rospy.Subscriber('neck_base/add_bulk_velpos', SetBulkVelPos, self.bulk_new_vel_pos)
        rospy.Subscriber('neck_base/add_bulk_timepos',SetBulkTimePos, self.bulk_new_time_pos)
        self.empty_positions_pub = rospy.Publisher('neck_base/empty_positions', UInt8, queue_size=12)
        
        rospy.Service('get_position', GetPosition, Ax12.get_present_position)
        rospy.Service('get_velocity',GetVelocity, Ax12.get_present_speed)
        rospy.loginfo("initialized")
        rospy.sleep(1)

        self.current_positions = {motor.id:motor.get_present_position() for motor in self.motors}

    def clear_motor(self, id):
        self.position_dict[id.data].clear()

    def clear_all(self,empty):
        for id in self.position_dict:
            self.position_dict[id] = []

    def get_next_velpos(self, id):
        next_velpos = self.position_dict[id].pop(0)
        if self.has_remaining_positions(id):
            self.empty_positions_pub.publish(id)
        return next_velpos
    
    def has_remaining_positions(self, id):
        return(bool(self.position_dict[id]))
    
        
    def main(self):
        
        self.ros_init()
        
        rospy.sleep(1)
        while not rospy.is_shutdown():
            moving = [motor.is_moving() for motor in self.motors]
            for i,motor in enumerate(self.motors):
                if not moving[i] and self.has_remaining_positions(i+1):
                    position, velocity = self.get_next_velpos(i + 1)
                    if velocity is not None:
                        motor.set_moving_speed(velocity)
                    motor.set_goal_position(position)
                    self.updated[i] = True
                elif not moving[i] and self.updated[i]:
                    self.current_positions[i+1] = motor.get_present_position()
                    self.updated[i] = False
                    

                    
            rospy.sleep(0.1)

if __name__ == "__main__":
    config_file = rospy.get_param('file', "../config/nyku_motor_config.yaml")
    port = rospy.get_param("NeckBase/port", "/dev/ttyUSB0")
    controller = NykuMotorController(config_file, port)
    controller.main()