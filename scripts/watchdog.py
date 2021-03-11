#!/usr/bin/env python

from __future__ import print_function

from std_msgs.msg import Float64MultiArray
import rospy
import vehicle_interface.srv
from vehicle_interface.msg import MapInfo
from geometry_msgs.msg import PoseArray, Pose, Transform




class Watchdog():
    def __init__ (self):
        self.stopped_tracking_service = rospy.Service('orbslam_alarm_watchdog_lost_tracking', vehicle_interface.srv.AlarmStoppedTracking, self.handle_AlarmStoppedTracking)
        self.map_merged = rospy.Service('orbslam_map_merged', vehicle_interface.srv.MapMergingInfo, self.handle_MapMergingInfo)
        rospy.wait_for_service('planner_reloc_request')
        self.caller_reloc_request = rospy.ServiceProxy('planner_reloc_request', vehicle_interface.srv.PlannerRelocRequest)
        self.caller_reloc_done = rospy.ServiceProxy('planner_reloc_done', vehicle_interface.srv.PlannerRelocDone)

        self.odom_sub=rospy.Subscriber("/map_info", MapInfo, self.map_info_callback)

        self.alarms=[]
        self.current_map=0
        self.desired_merges=[]


    def map_info_callback(self,msg):
        self.current_map=msg.current_map_id

    def handle_AlarmStoppedTracking(self,request):
        #########################################################
        #####  https://www.youtube.com/watch?v=suY06PVK_bI  #####
        #########################################################
        self.alarms.append(request)
        self.desired_merges.append(request.keyframe_map_id[0])
        score=0
        score_id=0
        for itr in range(len(request.data)):
            if request.keyframe_value[itr]>score: #what oder are frames put in the message? >= may be more desirable depending on that
                score_id=itr
                score=request.keyframe_value[itr]


        desired_pose=request.keyframe_value[score_id]
        reloc_transforms=[]
        reloc_transforms.append(request.data[score_id])
        
        #SEND MESSAGE TO PLANNER
        resp = self.caller_reloc_request(reloc_transforms)


        return vehicle_interface.srv.AlarmStoppedTrackingResponse()

    def handle_MapMergingInfo(self,request):
        #Go over alarms and reassign IDs (remove these alarms?)

        success=False

        for itr in range(len(desired_merges)):
            if desired_merges[itr]==request.merged_map_id:
                success=True
                del desired_merges[itr]
                break
        if success:
            resp=self.caller_reloc_done(request.merged_map_id)

        return vehicle_interface.srv.MapMergingInfo()



    # what do we do if relocalization fails? do we allow multiple maps?
    # smarter strategy for relocalization?



if __name__ == '__main__':
    rospy.init_node('slam_watchdog_node', anonymous=True)
    n=Watchdog()
    rospy.spin()        
#