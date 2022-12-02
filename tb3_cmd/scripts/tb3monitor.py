#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
from gazebo_msgs.srv import GetWorldProperties,GetModelState
from tb3_cmd.srv import GetClosest,GetClosestResponse,GetDistance, GetDistanceResponse

class GazeboUtils(object):
    def __init__(self):
        pass

    def getWorldProperties(self):
        try:
            get_world_properties=rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
            wp=get_world_properties()
            if wp.success:
                return wp
            else:
                rospy.logwarn(f"al invocar el servicio se recibio el estatus: {wp.success}")
                return None
        except rospy.rospy.ServiceException as se:
            rospy.logerr(f"Error al llamar [noimbre del servicio]: (se)")
    def getModelState(self,model_name, relative_entity_name='world'):
        try:
            get_model_state=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
            ms=get_model_state(model_name, relative_entity_name)
            if ms.success:
                return ms
            else:
                rospy.logwarn(f"al invocar el servicio se recibio el estatus {ms.success}")
                return None
        except rospy.rospy.ServiceException as se:
            rospy.logerr(f"Error al llamar [noimbre del servicio]: {se}")

class DistanceMonitor():
    def __init__(self):
        self._landmarks={}
        self._excluded_objects= ['ground_plane','turtlebot3_waffle']
        self._gazebo_utils= GazeboUtils()
        self.position=Point()
        self.odom_sub=rospy.Subscriber('/odom',Odometry, self._on_odom_callback)
        self.getClosestSrv=rospy.Service('/get_Closest',GetClosest,self.get_closest_srv)
        self.getDistanceSrv=rospy.Service('/get_distance',GetDistance, self.get_distance_srv)
        self._ini()

    def _ini(self):
        wp = self._gazebo_utils.getWorldProperties()
        if wp:
            for model in wp.model_names:
                if model not in self._excluded_objects:
                    ms = self._gazebo_utils.getModelState(model)
                    position = (ms.pose.position.x,ms.pose.position.y)
                    self._landmarks.update({model: position})

    def _on_odom_callback(self,msg):
        self.position=msg.pose.pose.position

    def get_closest_srv(self,msg):
        closest_landmark=''
        closest_distance=-1
        for model_name, (x,y) in self._landmarks.items():
            dx= x -self.position.x
            dy= y - self.position.y
            sqrt_dist= (dx*dx + dy*dy)
            if closest_distance == -1 or sqrt_dist < closest_distance:
                closest_distance=sqrt_dist
                closest_landmark = model_name
        response= GetClosestResponse()
        response.object_name = closest_landmark
        response.success=True
        response.status_message = "Todo Ok"

        return response


    
    def get_distance_srv(self,req):
        response=GetDistanceResponse
        if req.object_name not in self._landmarks:
            response.object_distance = 0.0 
            response.success = False
            response.status_message = f"El objeto '{req.object_name}' no fue encontraado"
            return response
        x,y = self._landmarks[req.object_name]
        dx= x -self.position.x
        dy= y - self.position.y
        response.object_distance = math.hypot(dx*dx + dy*dy)
        response.success=True
        response.status_message = "Todo ok"

        return response


def test_services():
    gazebo_utils=GazeboUtils()
    wp=gazebo_utils.getWorldProperties()
    if wp:
        for model in wp.model_names:
            ms=gazebo_utils.getModelState(model)
            position = (ms.pose.position.x,ms.pose.position.y)
            print(f"model_name: {model} (x:{position[0]}, y:{position[1]}) ")

def main():
    rospy.init_node('distancemonitorserver')
    monitor= DistanceMonitor()
    rospy.spin()

if __name__ =='__main__':
    main()