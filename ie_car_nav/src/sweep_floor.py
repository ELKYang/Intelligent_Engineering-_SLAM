#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib;
import rospy  
import actionlib  
import matplotlib.pyplot as plt
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  
from PIL import Image

msg = """
                    Sweeping
-------------------------------------------------
"""


class NavTest():  
    def __init__(self,region_num):  
        self.region_num=region_num
        rospy.init_node('exploring_slam', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  

        # 在每个目标位置暂停的时间 (单位：s)
        self.rest_time = rospy.get_param("~rest_time", 1)  

        # 是否仿真？  
        self.fake_test = rospy.get_param("~fake_test", True)  

        # 到达目标的状态  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']  
 
        # 设置目标点的位置  

        locations = dict()  
        # 扫厨房
        if self.region_num==1:
            locations['1'] = Pose(Point(-4.12,4.77, 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['2'] = Pose(Point(-1.37,4.64, 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['3'] = Pose(Point(-1.31,4.45, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['4'] = Pose(Point(-4.07,4.46, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['5'] = Pose(Point(-4,4, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['6'] = Pose(Point(-1.45,4, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['7'] = Pose(Point(-1.33,3.48, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
            locations['8'] = Pose(Point(0,0, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
        
        #扫厕所
        if self.region_num==2:
            locations['1'] = Pose(Point(0.67,4.7, 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['2'] = Pose(Point(-0.13,4.7, 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['3'] = Pose(Point(-0.13,4.2, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['4'] = Pose(Point(0.56,4.15, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['5'] = Pose(Point(0.56,3.7, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['6'] = Pose(Point(0.04,3.96, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['7'] = Pose(Point(0.668,3.22, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
            locations['8'] = Pose(Point(0,0, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
        #扫卧室
        if self.region_num==3:
            locations['1'] = Pose(Point(1.68,-0.12, 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['2'] = Pose(Point(2.14,4.42, 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['3'] = Pose(Point(2.68,-0.06, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['4'] = Pose(Point(2.74,4.35, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['5'] = Pose(Point(4.33,4.32, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['6'] = Pose(Point(4.26,-0.06, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['7'] = Pose(Point(3.78,-0.05, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
            locations['8'] = Pose(Point(3.76,2.27, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
            locations['9'] = Pose(Point(3.19,2.32, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
            locations['10'] = Pose(Point(3.16,0.2, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
            locations['11'] = Pose(Point(0,0, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
        
        #扫客厅
        if self.region_num==4:
            locations['1'] = Pose(Point(1.76,-1.36, 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['2'] = Pose(Point(1.76,-3.8, 0.000),  Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['3'] = Pose(Point(2.16,-3.8, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['4'] = Pose(Point(2.17,-1.37, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['5'] = Pose(Point(2.67,-1.37, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['6'] = Pose(Point(2.67,-3.8, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000))  
            locations['7'] = Pose(Point(3.17,-3.8, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
            locations['8'] = Pose(Point(3.17,-1.37, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
            locations['9'] = Pose(Point(3.67,-1.37, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
            locations['10'] = Pose(Point(3.67,-3.8, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
            locations['11'] = Pose(Point(4.17,-3.8, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
            locations['12'] = Pose(Point(4.17,-1.37, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
            locations['13'] = Pose(Point(0,0, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
        
        

        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  

        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  

        # 60s等待时间限制  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  
  
        # 保存机器人的在rviz中的初始位置  
        initial_pose = PoseWithCovarianceStamped()  

        # 保存成功率、运行时间、和距离的变量  
        n_locations = len(locations)  
        n_goals = 0  
        n_successes = 0  
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  
        location = ""  
        last_location = ""    
        i=0
        # 确保有初始位置  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)  

        rospy.loginfo("Starting Sweeping")  

        # 开始主循环，导航  
        while not rospy.is_shutdown():  
            if self.region_num==1:
                sequence=['1', '2', '3', '4', '5', '6', '7', '8']
            if self.region_num==2:
                sequence=['1', '2', '3', '4', '5', '6', '7', '8']
            if self.region_num==3:
                sequence=['1', '2', '3', '4', '5', '6', '7', '8','9','10','11']
            if self.region_num==4:
                sequence=['1', '2', '3', '4', '5', '6', '7', '8','9','10','11','12','13']
            
            
            if i==8 and self.region_num==1:
                rospy.loginfo("Swapping kitchen finished.")
                break
            if i==8 and self.region_num==2:
                rospy.loginfo("Swapping toilet finished.")
                break
            if i==11 and self.region_num==3:
                rospy.loginfo("Swapping bedroom finished.")
                break
            if i==13 and self.region_num==4:
                rospy.loginfo("Swapping living_room finished.")
                break

            location = sequence[i]  

            # 跟踪行驶距离  
            # 使用更新的初始位置  
            if initial_pose.header.stamp == "":  
                distance = sqrt(pow(locations[location].position.x -   
                                    locations[last_location].position.x, 2) +  
                                pow(locations[location].position.y -   
                                    locations[last_location].position.y, 2))  
            else:  
                #rospy.loginfo("Updating current pose.")  
                distance = sqrt(pow(locations[location].position.x -   
                                    initial_pose.pose.pose.position.x, 2) +  
                                pow(locations[location].position.y -   
                                    initial_pose.pose.pose.position.y, 2))  
                initial_pose.header.stamp = ""  

            # 存储上一次的位置，计算距离  
            last_location = location  

            # 计数器加1  
            i += 1  
            n_goals += 1  



            # 设定下一个目标点  
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = locations[location]  
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()  

            # 让用户知道下一个位置  
            #rospy.loginfo("Going to: " + str(location))  

            # 向下一个位置进发  
            self.move_base.send_goal(self.goal)  

            # 五分钟时间限制  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))   

            # 查看是否成功到达  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                #rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    #rospy.loginfo("Goal succeeded!")  
                    n_successes += 1  
                    distance_traveled += distance  
                    #rospy.loginfo("State:" + str(state))  
                else:  
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  

            # 运行所用时间  
            #running_time = rospy.Time.now() - start_time  
            #running_time = running_time.secs / 60.0  

            # 输出本次导航的所有信息  
            #rospy.loginfo("Success so far: " + str(n_successes) + "/" +   
                         # str(n_goals) + " = " +   
                         # str(100 * n_successes/n_goals) + "%")  

            #rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +   
                        #  " min Distance: " + str(trunc(distance_traveled, 1)) + " m")  

            rospy.sleep(self.rest_time)  

    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose  

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  

def trunc(f, n):  
    slen = len('%.*f' % (n, f))  

    return float(str(f)[:slen])  

if __name__ == '__main__':  
    try: 
        print msg
        region=input("请输入要打扫的区域的序号")
        NavTest(region)  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Sweeping finished.")
