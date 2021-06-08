#! /usr/bin/env python

import rospy
# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg._GoalStatus import GoalStatus

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from final_assignment.srv import RandomTarget, MoveBaseResult, MoveBaseResultResponse
from final_assignment.srv import RandomTargetResponse, MoveBaseTarget, MoveBaseTargetResponse
from std_srvs.srv import *
from geometry_msgs.msg import Twist


from math import *
import time


current_position_x = 0.0
current_position_y = 0.0
target = RandomTargetResponse()
state = 0


def the_distance_to_target(target):
    """
    Calculates the distance between the robot and the
    target
    Args:
        target (Object): Object containing the x and y
        coodinates of the new target
    Returns:
        (int): returns a tuple of the value of the
        distance to the target and the required yaw to
        face the direction of the target
    """

    dist_x = target.cord_x - current_position_x
    dist_y = target.cord_y - current_position_y
    distance_to_target = sqrt((dist_x * dist_x) + (dist_y * dist_y))
    return distance_to_target


def check_target(target):
    if (abs(current_position_x - target.cord_x) < 0.5 and abs(current_position_y - target.cord_y) < 0.5):
        return True
    else:
        return False


def handle_target(mes):
    global target
    global state
    if(check_target(mes)):
        return MoveBaseTargetResponse(True)
    else:
        target = mes
        state = 1
        return MoveBaseTargetResponse(False)


def handle_result(mes):
    time.sleep(1)
    while(state != 0):
        time.sleep(2)

    return MoveBaseResultResponse('Yes')


def feedback_callback(pose_message):
    """
    The pose callback function takes the position and posture of
    the robot from the argument "pose_message" and set it to
    three global variables containing the x, y and yaw position.
    Args:
        pose_message (Object): an object containing all the values
        of the current position and posture of the robot
    """

    # "global" makes the variable accessible everywhere in the code
    global current_position_x
    global current_position_y

    current_position = pose_message.base_position.pose.position

    current_position_x = current_position.x
    current_position_y = current_position.y


def movebase_client(client, target):

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"

    # Move to the target cordinate
    goal.target_pose.pose.position.x = target.cord_x
    goal.target_pose.pose.position.y = target.cord_y

    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=feedback_callback)
    # Waits for the server to finish performing the action.
    # wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
    # if not wait:
    #    rospy.logerr("Action server not available!")
    #    rospy.signal_shutdown("Action server not available!")
    return  # client.get_result()


def main():
    # Initializing the node
    rospy.init_node('movebase_client')
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rate = rospy.Rate(20)
    s = rospy.Service('movebase_client', MoveBaseTarget, handle_target)
    t = rospy.Service('movebase_result', MoveBaseResult, handle_result)
    global state
    while not rospy.is_shutdown():
        if (state == 0):
            rate.sleep()
            continue

        elif (state == 1):
            movebase_client(client, target)
            state = 2

        if (state == 2):
            time.sleep(0.5)
            distance_to_target = the_distance_to_target(target)
            print(
                f'Distance to target: {distance_to_target :.4f}, x: {current_position_x :.4f}, y: {current_position_y :.4f}')
            if (client.get_state() == GoalStatus.SUCCEEDED):
                print("\nTarget Reached !!!\n")
                state = 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
