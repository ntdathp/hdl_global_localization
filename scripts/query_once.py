#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from hdl_global_localization.srv import QueryGlobalLocalization, QueryGlobalLocalizationRequest

class HGLQueryOnce(object):
    def __init__(self):
        self.cloud_topic = rospy.get_param("~cloud_topic", "/points2/decompressed")
        self.max_num_candidates = rospy.get_param("~max_num_candidates", 1)

        self.latest_cloud = None
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)

        self.initialpose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1
        )

        rospy.loginfo("[HGLQueryOnce] Waiting for service /hdl_global_localization/query ...")
        rospy.wait_for_service("/hdl_global_localization/query")
        self.query_srv = rospy.ServiceProxy(
            "/hdl_global_localization/query", QueryGlobalLocalization
        )
        rospy.loginfo("[HGLQueryOnce] Service ready.")

        self.sent_query = False
        rospy.Timer(rospy.Duration(1.0), self.timer_cb)

    def cloud_cb(self, msg):
        self.latest_cloud = msg

    def timer_cb(self, event):
        if self.sent_query:
            return
        if self.latest_cloud is None:
            rospy.loginfo_throttle(
                2.0, "[HGLQueryOnce] Waiting for cloud on %s ..." % self.cloud_topic
            )
            return

        n_pts = self.latest_cloud.width * self.latest_cloud.height
        rospy.loginfo("[HGLQueryOnce] Current cloud has %d raw points", n_pts)

        req = QueryGlobalLocalizationRequest()
        req.max_num_candidates = self.max_num_candidates
        req.cloud = self.latest_cloud

        try:
            rospy.loginfo("[HGLQueryOnce] Calling /hdl_global_localization/query ...")
            resp = self.query_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr("[HGLQueryOnce] Service call failed: %s", str(e))
            self.sent_query = True
            return

        # ➜ từ đây trở xuống là chắc chắn service đã trả về
        rospy.loginfo(
            "[HGLQueryOnce] Resp: %d poses, %d errors, %d inlier_fractions",
            len(resp.poses),
            len(resp.errors),
            len(resp.inlier_fractions),
        )

        if len(resp.poses) == 0:
            rospy.logwarn("[HGLQueryOnce] No pose candidates returned.")
            self.sent_query = True
            return

        best_pose = resp.poses[0]
        best_err = resp.errors[0] if len(resp.errors) > 0 else -1.0
        best_inlier = (
            resp.inlier_fractions[0] if len(resp.inlier_fractions) > 0 else -1.0
        )

        rospy.loginfo(
            "[HGLQueryOnce] Got pose: err=%.3f, inlier=%.3f",
            best_err,
            best_inlier,
        )
        rospy.loginfo(
            "[HGLQueryOnce] Position: (%.3f, %.3f, %.3f)",
            best_pose.position.x,
            best_pose.position.y,
            best_pose.position.z,
        )

        init_msg = PoseWithCovarianceStamped()
        init_msg.header = resp.header  # thường là frame 'map'
        init_msg.pose.pose = best_pose
        init_msg.pose.covariance[0] = 0.25   # cov x
        init_msg.pose.covariance[7] = 0.25   # cov y
        init_msg.pose.covariance[35] = 0.1   # cov yaw

        self.initialpose_pub.publish(init_msg)
        rospy.loginfo("[HGLQueryOnce] Published /initialpose.")

        self.sent_query = True

def main():
    rospy.init_node("hgl_query_once")
    HGLQueryOnce()
    rospy.spin()

if __name__ == "__main__":
    main()
