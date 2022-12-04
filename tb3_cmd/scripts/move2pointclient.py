#!/usr/bin/env/ python3

from __future__ import print_function
import rospy
from tb3_cmd.srv import MovetoPoint,MovetoPointRequest,MovetoPointResponse
from geometry_msgs.msg import Pose2D
import sys


if __name__ == "__main__":
    req = MovetoPointRequest()
    response=MovetoPointResponse()
    if len(sys.argv) == 4 :
        req.target.x = float(sys.argv[1])
        req.target.y = float(sys.argv[2])
        req.target.theta = float(sys.argv[3])
    else:
        print("%s [z y theta]"%sys.argv[0])
        sys.exit(1)
    print(f"Enviando petición con meta x: {req.target.x} y: {req.target.y} theta: {req.target.theta}")
    rospy.wait_for_service('/move_2_point_service')
    try:
        move2point=rospy.ServiceProxy('/move_2_point_service',MovetoPoint)
        response=move2point(req)
        print("goal result")
        print(f"    x: {response.goal_result.x}")
        print(f"    y: {response.goal_result.y}")
        print(f"    theta: {response.goal_result.theta}")
        print(f"error_dist: {response.error_dist}")
        print(f"error orient: {response.error_orient}")
        print(f"duration: {response.duration.to_sec():.4f} segundos")
        print(f"status_message: {response.status_message}")
    except rospy.ServiceException as e:
        print("Error contactando al servicprint")
