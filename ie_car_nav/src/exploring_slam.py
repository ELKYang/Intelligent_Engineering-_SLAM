#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib;
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  

class NavTest():  
    def __init__(self):  
        rospy.init_node('exploring_slam', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  

        # 在每个目标位置暂停的时间 (单位：s)
        self.rest_time = rospy.get_param("~rest_time", 2)  

        # 是否仿真？  
        self.fake_test = rospy.get_param("~fake_test", True)  

        # 到达目标的状态  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']  
 
        # 设置目标点的位置  

        locations = dict()  
        
        locations['1'] = Pose(Point(1.96,0.28, 0.000),  Quaternion(0.000, 0.000, -0.447, 0.894))  
        locations['2'] = Pose(Point(4.49,0.47, 0.000),  Quaternion(0.000, 0.000, -0.847, 0.532))  
        locations['3'] = Pose(Point(4.27,4.44, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
        locations['4'] = Pose(Point(165,4.45, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764))  
        locations['5'] = Pose(Point(0.42,-1.74, 0.000), Quaternion(0.000, 0.000, 0.340, 0.940))  
        locations['6'] = Pose(Point(1.89,-3.77, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000))  
        locations['7'] = Pose(Point(2.6,-1, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
        locations['8'] = Pose(Point(0.526,1.76, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
        locations['9'] = Pose(Point(-0.113,4.21, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
        locations['10'] = Pose(Point(-0.19,-3.42, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
        locations['11'] = Pose(Point(-3.89,-3.41, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
        locations['12'] = Pose(Point(-3.91,3.68, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
        locations['13'] = Pose(Point(-1.08,3.73, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000)) 
        locations['14'] = Pose(Point(-1.5,-1.41, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000))
        
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

        rospy.loginfo("Starting navigation test")  

        # 开始主循环，导航  
        while not rospy.is_shutdown():  
            sequence=['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14']
            location = sequence[i]  

            # 跟踪行驶距离  
            # 使用更新的初始位置  
            if initial_pose.header.stamp == "":  
                distance = sqrt(pow(locations[location].position.x -   
                                    locations[last_location].position.x, 2) +  
                                pow(locations[location].position.y -   
                                    locations[last_location].position.y, 2))  
            else:  
                rospy.loginfo("Updating current pose.")  
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

            if i==15:
                rospy.loginfo("Exploring SLAM finished.")
                break

            # 设定下一个目标点  
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = locations[location]  
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()  

            # 让用户知道下一个位置  
            rospy.loginfo("Going to: " + str(location))  

            # 向下一个位置进发  
            self.move_base.send_goal(self.goal)  

            # 五分钟时间限制  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))   

            # 查看是否成功到达  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!")  
                    n_successes += 1  
                    distance_traveled += distance  
                    rospy.loginfo("State:" + str(state))  
                else:  
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  

            # 运行所用时间  
            running_time = rospy.Time.now() - start_time  
            running_time = running_time.secs / 60.0  

            # 输出本次导航的所有信息  
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +   
                          str(n_goals) + " = " +   
                          str(100 * n_successes/n_goals) + "%")  

            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +   
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")  

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
        NavTest()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring SLAM finished.")
