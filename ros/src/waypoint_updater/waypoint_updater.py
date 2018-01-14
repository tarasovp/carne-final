#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from copy import deepcopy
import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.vel_cb, queue_size=1)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb,queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        # TODO: Add other member variables you need below
        self.traffic = None
        
        # Run the iterations at 10 Hz
        rate = rospy.Rate ( 10 )
        while not rospy.is_shutdown ():
            if hasattr ( self, 'base_waypoints' ) and hasattr ( self, 'current_pose' ):
                l = Lane ()
                l.header.frame_id = '/world'
                l.header.stamp = rospy.Time.now ()

                now = np.argmin([(self.current_pose.pose.position.x-wp.pose.pose.position.x)**2+(self.current_pose.pose.position.y-wp.pose.pose.position.y)**2 
                    for wp in self.base_waypoints.waypoints])
            
                l.waypoints = [ self.base_waypoints.waypoints[(i+now)%len(self.base_waypoints.waypoints)] for i in range(LOOKAHEAD_WPS)]
                self.final_waypoints_pub.publish (l)
            rate.sleep ()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        rospy.loginfo('Trafic_light '+str(msg))
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def vel_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

            
    def traffic_cb(self, msg):
        '''
        Reads amd processes a traffic light signal
        '''
        rospy.loginfo(msg)
        if (msg.data >= 0):
            self.traffic = msg.data
        else:
            self.traffic = None

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
