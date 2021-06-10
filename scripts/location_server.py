#!/usr/bin/python3

"""
.. module:: location_server 
    :platform: Unix
    :synopsis: Selects a random target in a 3D Simulation
    
.. moduleauthor:: Omotoye Adekoya adekoyaomotoye@gmail.com  

This node selects a random target from the list of possible target of the environment
for mobile robots to navigate to. it accepts a request for a target and sends a response
of a randomly selected target. 
    
Service:
    /random_target to generate random target location  
    
"""


from final_assignment.srv import RandomTarget
from final_assignment.srv import RandomTargetResponse
from random import randint as rand
import rospy

TARGET_POSE = [(-4, -3), (-4, 2), (-4, 7), (5, -7), (5, -3), (5, 1)]


def handle_random_target(req):
    """
    Takes in a request message and returns a random target
    coordinte, x and y

    Args:
        req (str): The request message sent by the Client

    Returns:
        [Object]: The response sent to the Client 
    """

    # Random Location selector
    rand_index = rand(0, 5)
    rand_x, rand_y = TARGET_POSE[rand_index]
    print(f'The Target Location is x: {rand_x}, y: {rand_y}')
    return RandomTargetResponse(rand_x, rand_y)


def random_target_gen():
    """
    Initializes the Service and sends request message to
    the callback function
    """
    rospy.init_node('random_target_gen')
    s = rospy.Service('random_target', RandomTarget, handle_random_target)
    rospy.spin()


if __name__ == '__main__':
    random_target_gen()
