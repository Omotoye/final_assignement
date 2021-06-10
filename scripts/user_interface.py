#! /usr/bin/env python

import rospy

from final_assignment.srv import RandomTarget
from final_assignment.srv import RandomTargetResponse
from final_assignment.srv import MoveBaseTarget, MoveBaseResult
from std_srvs.srv import *
from geometry_msgs.msg import Twist

from math import *
import time

TARGET_POSE = [(-4, -3), (-4, 2), (-4, 7), (5, -7), (5, -3), (5, 1)]
target2 = RandomTargetResponse()
algo = 0
path_planner = ['Move Base: Dijkstra', 'Bug0']
prompt_mes = f"""
Please Enter a number corresponding to one of the following actions:

Possible Position = {TARGET_POSE}
Path Planning Algorithm: {path_planner[algo]}

1. Move to a Random position from one of the 6 possible positions.
2. Select one of the possible positions.
3. Start following external walls.
4. Stop in the last position. 
5. Change the planning Algorithm from Dijkstra to Bug0

"""


def check_location(x, y):
    if (x, y) in TARGET_POSE:
        return True
    else:
        return False


def call_rand_target():
    rospy.wait_for_service('random_target')
    try:
        random_target = rospy.ServiceProxy('random_target', RandomTarget)
        target = random_target('Waiting for Target')
    except rospy.ServiceException as e:
        print(f'Service call failed: {e}')
    return target


def call_movebase(target):
    rospy.wait_for_service('movebase_client')
    try:
        movebase_client = rospy.ServiceProxy('movebase_client', MoveBaseTarget)
        res = movebase_client(target.cord_x, target.cord_y)
        return res.status
    except rospy.ServiceException as e:
        print(f'Service call failed: {e}')


def call_bug_algo(target):
    rospy.set_param("des_pos_x", target.cord_x)
    rospy.set_param("des_pos_y", target.cord_y)
    print("Thanks! Let's reach the next position")
    rospy.wait_for_service('bug_switch')
    try:
        bug0 = rospy.ServiceProxy('bug_switch', MoveBaseResult)
        res = bug0('Are you there yet!')
        return res.status
    except rospy.ServiceException as e:
        print(f'Service call failed: {e}')


def wait_for_result():
    rospy.wait_for_service('movebase_result')
    try:
        movebase_result = rospy.ServiceProxy('movebase_result', MoveBaseResult)
        res = movebase_result('Are you there yet!')
        return res.status
    except rospy.ServiceException as e:
        print(f'Service call failed: {e}')


def call_wall_follower(msg):
    rospy.wait_for_service('/wall_follower_switch')
    try:
        srv_client_wall_follower_ = rospy.ServiceProxy(
            '/wall_follower_switch', SetBool)
        resp = srv_client_wall_follower_(msg)
        return True
    except rospy.ServiceException as e:
        print(f'Service call failed: {e}')


def main():
    # Initializing the node
    rospy.init_node('user_interface')

    # Creating a publisher object
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    global algo, prompt_mes
    state = 0
    pick = 0

    while not rospy.is_shutdown():
        if (state == 0):
            print(prompt_mes)
            try:
                pick = int(input("Enter the number here: "))
                state = 1
            except ValueError:
                print('\nPLEASE ENTER A VALID NUMBER !!!\n')
                state = 0

        if (pick == 1 and state == 1):
            target = call_rand_target()
            if (algo == 0):
                resp = call_movebase(target)
                while (resp):
                    target = call_rand_target()
                    resp = call_movebase(target)
                wait_for_result()
                state = 0
            else:
                resp = call_bug_algo(target2)
                print(resp)
                algo = 0 
                state = 0

        elif (pick == 2 and state == 1):
            print('Please enter a x and y cordinate from the possible position list')
            x = int(input('x: '))
            y = int(input('y: '))
            if (check_location(x, y)):
                target2.cord_x = x
                target2.cord_y = y
                if (algo == 0):
                    resp = call_movebase(target2)
                    if(resp):
                        print('The Robot is already at this location')
                    else:
                        wait_for_result()
                        state = 0
                else: 
                    resp = call_bug_algo(target2)
                    print(resp)
                    algo = 0 
                    state = 0
            else:
                print('Please enter one of the possible positions')

        elif (pick == 3 and state == 1):
            call_wall_follower(True)
            print('Follow Wall has Started')
            state = 0
        elif (pick == 4 and state == 1):
            call_wall_follower(False)
            # Creating an object for the robot motion
            velocity = Twist()
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            vel_pub.publish(velocity)
            print('Follow wall has been stopped')
            state = 0
        elif (pick == 5 and state == 1):
            algo = 1 
            print('The path planning algorithm has been changed to Bug0')
            state = 0 
        elif (pick not in range(1, 6)):
            state = 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
