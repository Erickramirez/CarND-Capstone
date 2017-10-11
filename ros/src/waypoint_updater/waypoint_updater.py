#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from   std_msgs.msg      import Int32, Float32

import math
import tf
from platform import dist


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
MINIMUM_DISTANCE_FROM_LIGHT = 2
MAX_VELOCITY = 10

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.cur_velocity_cb)
        self.base_waypoints_sub =rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint',  Int32,       self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        # TODO: Add other member variables you need below
        self.next_wp_idx= -1
        self.cur_pose = None
        self.waypoints = None
        self.current_velocity = 0
        self.red_light_index = -1
        self.isbraking = False
        self.too_close = False



        #rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.waypoints is not None:
                #rospy.loginfo("red light =" + str(self.next_wp_idx))
                self.next_wp_idx = self.get_next_waypoint_idx()
                #rospy.loginfo("Next Waypoint Idx: %s", self.next_wp_idx)
                next_waypoints = self.waypoints
                next_waypoints = self.waypoints[self.next_wp_idx:(self.next_wp_idx+LOOKAHEAD_WPS)]
                if self.red_light_index != -1:
                    dist = self.distance(self.waypoints, self.next_wp_idx,self.red_light_index)
                    rospy.loginfo("POP " + str(dist))
                    if dist > MINIMUM_DISTANCE_FROM_LIGHT:
                        distance_to_stop = dist - MINIMUM_DISTANCE_FROM_LIGHT
                        for i, wp in enumerate(next_waypoints):
                            if self.distance(next_waypoints,0,i) > distance_to_stop:
                                self.set_waypoint_velocity(next_waypoints,i,0)
                            velocity = max(self.current_velocity,1)
                            velocity -= 10
                            self.set_waypoint_velocity(next_waypoints,i,max(0,velocity))
                    else:
                        for i, wp in enumerate(next_waypoints):
                            self.set_waypoint_velocity(next_waypoints,i,0)
                        self.too_close = False
                else:
                    if self.current_velocity < MAX_VELOCITY:
                        dist = 10
                        for i, wp in enumerate(next_waypoints):
                            velocity = self.current_velocity
                            velocity += 2
                            self.set_waypoint_velocity(next_waypoints,i,min(MAX_VELOCITY,velocity))

                    
                lane = Lane()
                lane.waypoints = next_waypoints
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time(0)
                self.final_waypoints_pub.publish(lane)
                if len(next_waypoints) > 4:
                    rospy.loginfo("Next Waypoint: %s \n 5th: %s\nCurrent Pose: %s", next_waypoints[0].pose.pose, next_waypoints[5].pose.pose,self.cur_pose)

            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        self.cur_pose = msg.pose
        #rospy.loginfo("Current Pose: %s", self.cur_pose)
    def cur_velocity_cb(self, velocity):
        self.current_velocity = velocity.twist.linear.x


    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #Samer: waypoints is always the same so only need to load once
        if self.waypoints is None:
            self.waypoints = waypoints.waypoints + waypoints.waypoints

            #for i in range(0,len(self.waypoints)):
                #self.set_waypoint_velocity(self.waypoints,i,40)

            self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.red_light_index = msg.data
        rospy.loginfo("Index of waypoints red light " + str(self.red_light_index))
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def needs_braking(self,nextwaypoint_index,distance):
        return nextwaypoint_index > self.red_light_index and distance >MINIMUM_DISTANCE_FROM_LIGHT


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

    def get_vector_from_quaternion(self, q):
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        x = math.cos(yaw) * math.cos(pitch)
        y = math.sin(yaw) * math.cos(pitch)
        z = math.sin(pitch)
        return x, y, z

    def get_next_waypoint_idx(self):

        if self.waypoints is None or self.cur_pose is None:
            return -1

        cur_x = self.cur_pose.position.x
        cur_y = self.cur_pose.position.y

        #finding closest waypoint
        num_wp = len(self.waypoints)/2 + 5
        min_dist = -1
        closest_wp_idx = -1
        for i in range(0, num_wp):
            wp_x = self.waypoints[i].pose.pose.position.x
            wp_y = self.waypoints[i].pose.pose.position.y

            dist = math.sqrt((wp_x-cur_x)**2 + (wp_y-cur_y)**2)

            if min_dist == -1 or dist < min_dist:
                min_dist = dist
                closest_wp_idx = i


        #getting closest waypoint in front of vehicle
        while(1):
            closest_wp = self.waypoints[closest_wp_idx]
            closest_wp_x = closest_wp.pose.pose.position.x
            closest_wp_y = closest_wp.pose.pose.position.y

            x_vec, y_vec,z_vec = self.get_vector_from_quaternion(self.cur_pose.orientation)

            vec_dist =  math.sqrt((closest_wp_x-cur_x - x_vec*0.1)**2 + (closest_wp_y-cur_y - y_vec*0.1)**2)

            closest_wp_idx+=1

            if vec_dist > min_dist:
                break
            #else:
                #return closest_wp_idx + 1

        return closest_wp_idx


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
