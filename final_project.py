from __future__ import print_function
from tkinter.tix import X_REGION
from six.moves import input
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from scipy.spatial.transform import Rotation

import numpy as np
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import math
from tm_msgs.msg import *
from tm_msgs.srv import *

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("team14_move_group", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        ## This interface can be used to plan and execute motions:
        group_name = "tmr_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def plan_cartesian_path(self, x=0.0, y=0.0, scale=0.0005):
        move_group = self.move_group
        wpose = move_group.get_current_pose().pose

        waypoints = []

        wpose.position.x -= scale * y
        wpose.position.y -= scale * x

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        move_group.execute(plan, wait=True)

    def set_joint_state(self, a):

        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = a[0]/(180/pi)
        joint_goal[1] = a[1]/(180/pi)
        joint_goal[2] = a[2]/(180/pi)
        joint_goal[3] = a[3]/(180/pi)
        joint_goal[4] = a[4]/(180/pi)
        joint_goal[5] = a[5]/(180/pi)

        move_group.go(joint_goal, wait=True)
        move_group.stop()

    def set_angular_joint_state(self, i, theta):

        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        current_joints = move_group.get_current_joint_values()

        # base = current_joints[0]*180/pi

        # if theta > 0:
        #     angle = -90 + theta + base
        # elif theta <= 0:
        #     angle = 90 + theta + base

        # print(angle)
        joint_goal[i] = theta/(180/pi)

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        
        return all_close(joint_goal, current_joints, 0.01)

    def set_pose_goal(self, x, y, z):   
        
        print("you enter:", x, y, z)
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 1 / math.sqrt(2)
        pose_goal.orientation.y = 1 / math.sqrt(2)
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def set_cartesian_path(self, p, scale = 0.0005):
        move_group = self.move_group
        wpose = move_group.get_current_pose().pose

        waypoints = []

        wpose.position.x += scale * p[0]
        wpose.position.y += scale * p[1]
        wpose.position.z += scale * p[2]
        wpose.orientation.x = p[3]
        wpose.orientation.y = p[4]
        wpose.orientation.z = p[5]
        wpose.orientation.w = p[6]
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        move_group.execute(plan, wait=True)
        

    def check_position(self):

        move_group = self.move_group

        wpose = move_group.get_current_pose().pose
        x = wpose.position.x
        y = wpose.position.y
        z = wpose.position.z
        a = wpose.orientation.x
        b = wpose.orientation.y
        c = wpose.orientation.z
        d = wpose.orientation.w
        # print('position:', x, y, z)
        # print('rotation:', a, b, c, d)

        position = [x, y, z, a, b, c, d]
        return position

    def check_angular(self):
        move_group = self.move_group
        current_joints = move_group.get_current_joint_values()
        a1 = current_joints[0]/pi*180
        a2 = current_joints[1]/pi*180
        a3 = current_joints[2]/pi*180
        a4 = current_joints[3]/pi*180
        a5 = current_joints[4]/pi*180
        a6 = current_joints[5]/pi*180

        return [a1, a2, a3, a4, a5, a6]

    def get_data(self, data):
        
        global dx
        global dy
        global da

        dx = data.data[0] - 320
        dy = data.data[1] - 240
        da = data.data[2]


    
    def printout(self, data):

        x = data.data[0]
        y = data.data[1]
        a = data.data[2]
        # print(x, y, a)

        global stable

        check_x.append(int(x))
        if check_x[-1] == check_x[-2] & check_x[-1] == check_x[-3]:
            stable = True
    
    def angular_joint_state(self, theta):

        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        current_joints = move_group.get_current_joint_values()

        base = current_joints[0]*180/pi

        if theta > 0:
            angle = theta + base
        elif theta <= 0:
            angle = theta + base

        print(angle)
        joint_goal[5] = angle/(180/pi)

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        
        return all_close(joint_goal, current_joints, 0.01)


def relative_displacement(p1,p2):

    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    dz = p1[2] - p2[2]
    return [dx, dy, dz, p1[3], p1[4], p1[5], p1[6]]


check_x = [0,0]
stable = False
dx, dy, da = 0, 0, 0
IDS = []
start = 0

wait_time = 0.8
srqt2 = math.sqrt(2)

# define joint state
s1 = [38, -12, 100, 1, 90, 38]
ready1 = [0, 0, 90, 0, 90, 0]
ready2 = [0, 0, 90, -90, 90, 0]

def get_ID(data):
    global IDS
    IDS= data.data

def is_start(data):
    global start
    start = data.data



def find_card(tutorial, iter):
    print("==============start==============")
    tutorial.set_joint_state(s1)
    while(1):
        a1, a2, a3, a4, a5, a6 = tutorial.check_angular()
        if abs(a1 - s1[0]) < 0.01:
            time.sleep(wait_time)
            break

    print("==============move to center==============")
    reach = False
    while(reach == False):
        
        pub_id.publish(Int32(data = IDS[iter-1]))
        rospy.sleep(0.2)
        rospy.Subscriber("aruco_xya", Float32MultiArray, tutorial.get_data)
        tutorial.plan_cartesian_path(x = dx, y = dy, scale = 0.0005)
        rospy.sleep(wait_time*1.5)
        print('distance: ', dx, dy)
        
        if abs(dx) + abs(dy) < 8:
            reach = True

    print("==============gripper to center==============")
    p = tutorial.check_position()
    dp = [0.125, -0.0125, 0] + p[-4:]
    tutorial.set_cartesian_path(dp, 1)
    while(1):
        pp = tutorial.check_position()
        if abs(pp[0] - p[0] - 0.125) < 0.01:
            time.sleep(wait_time)
            break
   

    print("==============rotate==============")
    rospy.Subscriber("aruco_xya", Float32MultiArray, tutorial.get_data)
    print(da)
    tutorial.angular_joint_state(da)
    time.sleep(wait_time*2)


    print("==============gripper down==============")
    p = tutorial.check_position()
    dp = [0, 0, -0.355] + p[-4:] # -0.343
    tutorial.set_cartesian_path(dp, 1)
    while(1):
        pp = tutorial.check_position()
        if abs(pp[2] - p[2] + 0.355) < 0.01:
            time.sleep(wait_time)
            break



def flip_card(tutorial, iter, degree):
    print("==============start==============")
    tutorial.set_joint_state(s1)
    while(1):
        a = tutorial.check_angular()
        if abs(a[2] - s1[2]) < 0.01:
            time.sleep(wait_time*0.5)
            break
    
    print("==============go to box==============")
    tutorial.set_joint_state(ready1)
    while(1):
        a = tutorial.check_angular()
        if abs(a[3] - 0) < 0.01:
            time.sleep(wait_time)
            break
    
    if(iter==2):
        h = tutorial.check_position()[1]
        dp = [0, -0.08, 0, 1/srqt2, 1/srqt2, 0, 0]
        tutorial.set_cartesian_path(dp, 1)
        h -= 0.08
        while(1):
            p = tutorial.check_position()
            if abs(p[1] - h) < 0.01:
                time.sleep(wait_time)
                break
        
    h = tutorial.check_position()[2]
    dp = [0, 0, -0.025, 1/srqt2, 1/srqt2, 0, 0]
    tutorial.set_cartesian_path(dp, 1)
    h -= 0.025
    while(1):
        p = tutorial.check_position()
        if abs(p[2] - h) < 0.01:
            time.sleep(wait_time)
            break
            
    h = tutorial.check_position()[2]
    print("==============flip the card==============")
    dp = [-0.2, 0, -0.04, 0.5, 0.5, 0.5, 0.5]
    tutorial.set_cartesian_path(dp, 1)
    h -= 0.04
    while(1):
        p = tutorial.check_position()
        if abs(p[2] - h) < 0.01:
            time.sleep(wait_time*2)
            break
    
    # on step further down
    h = tutorial.check_position()[2]
    dp = [0, 0, -0.065, 0.5, 0.5, 0.5, 0.5]
    tutorial.set_cartesian_path(dp, 1)
    h -= 0.065
    while(1):
        p = tutorial.check_position()
        if abs(p[2] - h) < 0.01:
            time.sleep(wait_time*2)
            break
    
    h = tutorial.check_position()[2]
    dp = [0, 0, 0.2, 0.5, 0.5, 0.5, 0.5]
    tutorial.set_cartesian_path(dp, 1)
    h += 0.2
    while(1):
        p = tutorial.check_position()
        if abs(p[2] - h) < 0.01:
            time.sleep(wait_time)
            break
    
    print("==============turn 180 degree==============")
    tutorial.set_angular_joint_state(5, degree)
    while(1):
        a = tutorial.check_angular()
        if abs(a[5] - degree) < 0.01:
            time.sleep(wait_time)
            break
    
    print("==============magical angle==============")
    d = 0.1
    q = [-0.5, 0.5, -0.5, 0.5]
    q[0] -= d
    q[1] += d
    q[2] = -math.sqrt((1 - (q[0]**2)*2)/2)
    q[3] = math.sqrt((1 - (q[0]**2)*2)/2)
    dp = [0, 0, 0] + q
    tutorial.set_cartesian_path(dp, 1)
    while(1):
        p = tutorial.check_position()
        if abs(p[3] + 0.5 + d) < 0.01:
            time.sleep(wait_time)
            break
    
    # dowm and back
    h = tutorial.check_position()[2]
    dp = [-0.03, 0, -0.18] + q
    tutorial.set_cartesian_path(dp, 1)
    h -= 0.18
    while(1):
        p = tutorial.check_position()
        if abs(p[2] - h) < 0.01:
            time.sleep(wait_time)
            break
    
    h = tutorial.check_position()[2]
    dp = [0.06, 0, 0.06] + [1/math.sqrt(2), -1/math.sqrt(2), 0, 0]
    tutorial.set_cartesian_path(dp, 1)
    h += 0.06
    while(1):
        p = tutorial.check_position()
        if abs(p[2] - h) < 0.01:
            time.sleep(wait_time)
            break
    
    print("==============front and back==============")
    dp = [0.035, 0, 0] + [1/math.sqrt(2), -1/math.sqrt(2), 0, 0]
    tutorial.set_cartesian_path(dp, 1)
    time.sleep(wait_time*3)
    dp = [-0.035, 0, 0] + [1/math.sqrt(2), -1/math.sqrt(2), 0, 0]
    tutorial.set_cartesian_path(dp, 1)
    time.sleep(wait_time*3)

    print("==============finish==============")
    dp = [0, 0, 0.2] + [1/math.sqrt(2), -1/math.sqrt(2), 0, 0]
    tutorial.set_cartesian_path(dp, 1)
    time.sleep(wait_time*3)

    tutorial.set_joint_state(s1)
    while(1):
        a = tutorial.check_angular()
        if abs(a[3] - s1[3]) < 0.01:
            time.sleep(wait_time)
            break



def main():
    tutorial = MoveGroupPythonInterfaceTutorial()
    
    try:
        
        tutorial.set_joint_state(s1)
        while(1):
            global start
            rospy.Subscriber("robot_start", Int32, is_start)
            if(start==1):
                pub_end.publish(0)
                print('is robot')
                rospy.Subscriber("id_talker", Int32MultiArray, get_ID)
                find_card(tutorial, 1)
                flip_card(tutorial, 1, 180)
                find_card(tutorial, 2)
                flip_card(tutorial, 2, 170)
                rospy.sleep(5)
                
                pub_end.publish(1)
            
            start = 0
        




    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    pub_end = rospy.Publisher("move_end", Int32)
    pub_id = rospy.Publisher("move_group_id", Int32)
    main()

