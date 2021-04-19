#!/usr/bin/env python2

from __future__ import print_function

from std_msgs.msg import Float64MultiArray
import rospy
import vehicle_interface.srv
from vehicle_interface.msg import MapInfo
from vehicle_interface.srv import RelocalisationRequest
from geometry_msgs.msg import PoseArray, Pose, Transform, PoseStamped
from nav_msgs.msg import Odometry
from pose_graph_msgs.msg import GraphSE3, VertexSE3
import heapq
import numpy


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
        # rospy.wait_for_service('/bluerov2_rosplan/failures/planner_reloc_request')
        self.caller_reloc_request = rospy.ServiceProxy('/bluerov2_rosplan/failures/planner_reloc_request', vehicle_interface.srv.PlannerRelocRequest)
        self.caller_reloc_done = rospy.ServiceProxy('/bluerov2_rosplan/failures/planner_reloc_done', vehicle_interface.srv.PlannerRelocDone)
        self.get_relocalisation_path_srv = rospy.ServiceProxy('/bluerov2/frontier/relocalisation_path', vehicle_interface.srv.RelocalisationRequest)
        self.stopped_tracking_service = rospy.Service("/ORBSLAM3/lost", vehicle_interface.srv.AlarmStoppedTracking, self.handle_AlarmStoppedTracking)
        rospy.loginfo("alarm stopped tracking service started")
        self.lost_tracking_sub=rospy.Subscriber("/ORBSLAM3/map_info", MapInfo, self.map_info_callback)
        self.pose_sub=rospy.Subscriber("/orb_pose", PoseStamped, self.pose_cb)

        
        
        self.graph_pub=rospy.Publisher("/bluerov2/watchdog/keyframes_sent", GraphSE3, queue_size=10)
        self.pa_sent_pub=rospy.Publisher("/bluerov2/watchdog/pose_arrays_sent", PoseArray, queue_size=10)
        self.pa_received_pub=rospy.Publisher("/bluerov2/watchdog/pose_arrays_received", PoseArray, queue_size=10)

        rospy.loginfo("watchdog initialised")

    def pose_cb(self, msg):
        self.last_pose = msg
        self.has_pose=True

    def map_info_callback(self,msg):
        self.current_map=msg.current_map_id

    def handle_AlarmStoppedTracking(self,request):
        rospy.loginfo("received call")

        graph_msg = GraphSE3()

        #########################################################
        #####  https://www.youtube.com/watch?v=suY06PVK_bI  #####
        #########################################################
        self.alarms.append(request)
        self.desired_merges.append(request.keyframe_map_id[0])
        score=0
        score_id=0
        good_keyframes = []
        rospy.loginfo("Received %d keyframes, nLast = %d", len(request.data), self.n_last)
        for itr in range(len(request.data)):
            keyframe_pose = request.data[itr]
            keyframe_value = request.keyframe_value[itr]
            keyframe_vertex = VertexSE3()
            keyframe_vertex.pose.position.x = keyframe_pose.translation.x
            keyframe_vertex.pose.position.y = keyframe_pose.translation.y
            keyframe_vertex.pose.position.z = keyframe_pose.translation.z
            keyframe_vertex.pose.orientation.x = keyframe_pose.rotation.x
            keyframe_vertex.pose.orientation.y = keyframe_pose.rotation.y
            keyframe_vertex.pose.orientation.z = keyframe_pose.rotation.z
            keyframe_vertex.pose.orientation.w = keyframe_pose.rotation.w
            keyframe_vertex.robustness = keyframe_value
            graph_msg.vertices.append(keyframe_vertex)
            if request.keyframe_value[itr]>=score: #what oder are frames put in the message? >= may be more desirable depending on that
                score_id=itr
                score=request.keyframe_value[itr]


        keyframe_values = numpy.array(request.keyframe_value)
        good_keyframes = numpy.sort(heapq.nlargest(min(self.n_last,len(keyframe_values)), range(len(keyframe_values)), keyframe_values.take))

        self.graph_pub.publish(graph_msg)
        desired_pose=request.keyframe_value[score_id]
        reloc_transforms=[]
#        reloc_transforms.append(request.data[score_id])
        
        if self.has_pose or True: #or for now
            # rr = RelocalisationRequest()
            # rr.samples = self.n_samples
            # rr.n_last = self.n_last
            # rr.key_frames = request.data[0:min(self.n_last, len(request.data))]
            # rr.start.pose.pose = self.last_pose.pose
            PA = PoseArray()
            PA.header.frame_id = "orb_slam"
            for keyframe_id in good_keyframes:
                P = Pose()
                P.position.x, P.position.y, P.position.z = request.data[keyframe_id].translation.x, request.data[keyframe_id].translation.y, request.data[keyframe_id].translation.z
                P.orientation.x, P.orientation.y, P.orientation.z, P.orientation.w = request.data[keyframe_id].rotation.x, request.data[keyframe_id].rotation.y, request.data[keyframe_id].rotation.z, request.data[keyframe_id].rotation.w
                PA.poses.append(P)

            self.pa_sent_pub.publish(PA)
            rospy.loginfo("Size of poseArray sent to planner is %d", len(PA.poses))
            start_odom = Odometry()
            start_odom.pose.pose = self.last_pose.pose
            rr_resp = self.get_relocalisation_path_srv.call(self.n_samples, self.n_last, PA, start_odom)
#            reloc_transforms=[]
            # print (rr_resp.found_path)
            PA_received = PoseArray()
            PA_received.header.frame_id = "orb_slam"
            for i in range(len(rr_resp.found_path.poses)):
                PA_received.poses.append(rr_resp.found_path.poses[i].pose)
                new_pose = Transform()
                new_pose.translation.x, new_pose.translation.y, new_pose.translation.z = rr_resp.found_path.poses[i].pose.position.x, rr_resp.found_path.poses[i].pose.position.y, rr_resp.found_path.poses[i].pose.position.z
                new_pose.rotation.x, new_pose.rotation.y, new_pose.rotation.z, new_pose.rotation.w = rr_resp.found_path.poses[i].pose.orientation.x, rr_resp.found_path.poses[i].pose.orientation.y, rr_resp.found_path.poses[i].pose.orientation.z,  rr_resp.found_path.poses[i].pose.orientation.w
                reloc_transforms.append(new_pose)
            self.pa_received_pub.publish(PA_received)
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