#!/usr/bin/env python

from __future__ import print_function

    from std_msgs.msg import Float64MultiArray
import rospy
import vehicle_interface.srv
from vehicle_interface.msg import MapInfo
from geometry_msgs.msg import PoseArray, Pose, Transform, PoseStamped




class Watchdog():
    def __init__ (self):
        rospy.loginfo("starting init")
        

        self.alarms=[]
        self.current_map=0
        self.n_last = 5
        self.n_samples = 100
        self.has_pose = False
        self.last_pose = PoseStamped
        self.desired_merges=[]
        self.map_merged = rospy.Service('/ORBSLAM3/map_merged', vehicle_interface.srv.MapMergingInfo, self.handle_MapMergingInfo)
        rospy.wait_for_service('/bluerov2_rosplan/failures/planner_reloc_request')
        self.caller_reloc_request = rospy.ServiceProxy('/bluerov2_rosplan/failures/planner_reloc_request', vehicle_interface.srv.PlannerRelocRequest)
        self.caller_reloc_done = rospy.ServiceProxy('/bluerov2_rosplan/failures/planner_reloc_done', vehicle_interface.srv.PlannerRelocDone)
        self.get_relocalisation_path_srv = rospy.ServiceProxy('/bluerov2/frontier/relocalisation_path', vehicle_interface.srv.RelocalisationRequest)
        self.stopped_tracking_service = rospy.Service("/ORBSLAM3/lost", vehicle_interface.srv.AlarmStoppedTracking, self.handle_AlarmStoppedTracking)
        rospy.loginfo("alarm stopped tracking s ervice started")
        self.lost_tracking_sub=rospy.Subscriber("/ORBSLAM3/map_info", MapInfo, self.map_info_callback)
        self.pose_sub=rospy.Subscriber("/orb_pose", PoseStamped, self.pose_cb)
        
        
        rospy.loginfo("watchdog initialised")

    def pose_cb(self, msg):
        self.last_pose = msg
        self.has_pose=True

    def map_info_callback(self,msg):
        self.current_map=msg.current_map_id

    def handle_AlarmStoppedTracking(self,request):
        rospy.loginfo("received call")

        #########################################################
        #####  https://www.youtube.com/watch?v=suY06PVK_bI  #####
        #########################################################
        self.alarms.append(request)
        self.desired_merges.append(request.keyframe_map_id[0])
#        score=0
#        score_id=0
#        for itr in range(len(request.data)):
#            if request.keyframe_value[itr]>score: #what oder are frames put in the message? >= may be more desirable depending on that
#                score_id=itr
#                score=request.keyframe_value[itr]


#        desired_pose=request.keyframe_value[score_id]
        reloc_transforms=[]
#        reloc_transforms.append(request.data[score_id])
        
        if self.has_pose or True: #or for now
            rr = RelocalisationRequest()
            rr.samples = self.n_samples
            rr.n_last = self.n_last
            rr.key_frames = request.data[0:min(self.n_last, len(request.data))]
            rr.start.pose = self.last_pose
            rr_resp = self.get_relocalisation_path_srv.call(rr)
#            reloc_transforms=[]
            for i in range(len(rr_resp.found_path)):
                new_pose = geometry_msgs.Transform;
                new_pose.translation.x, new_pose.translation.y, new_pose.translation.z = rr_resp.found_path.poses[i].pose.position.x, rr_resp.found_path.poses[i].pose.position.y, rr_resp.found_path.poses[i].pose.position.z
                new_pose.rotation.x, new_pose.rotation.y, new_pose.rotation.z, new_pose.rotation.w = rr_resp.found_path.poses[i].pose.orientation.x, rr_resp.found_path.poses[i].pose.orientation.y, rr_resp.found_path.poses[i].pose.orientation.z,  rr_resp.found_path.poses[i].pose.orientation.w
                reloc_transforms.append(new_pose)
        
        # desired_pose=request.keyframe_value[0]
        # reloc_transforms=[]
        # reloc_transforms.append(request.data[0])
        
        #SEND MESSAGE TO PLANNER
        rospy.loginfo("sending transforms to planner")
        try:
            resp = self.caller_reloc_request(reloc_transforms)
        except Exception() as e:
            rospy.loginfo(e)
        rospy.loginfo("=====================================================================")
        return resp
        # return vehicle_interface.srv.AlarmStoppedTrackingResponse()

    def handle_MapMergingInfo(self,request):
        #Go over alarms and reassign IDs (remove these alarms?)
        rospy.loginfo("Received map merged service call")
        success=False

        for itr in range(len(self.desired_merges)):
            if self.desired_merges[itr]==request.merged_map_id:
                success=True
                del self.desired_merges[itr]
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