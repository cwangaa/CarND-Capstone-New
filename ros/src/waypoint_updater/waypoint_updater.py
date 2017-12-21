#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from std_msgs.msg import Int32
import math

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
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        print("Subscriptions completed")
        self.count = 0
        #
        # Define a place to store the waypoints
        self.wps = None
        # Define a variable to define the previous waypoint of the car
        self.wp_index = None
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.speed_limit =  rospy.get_param('/waypoint_loader/velocity', 80.46)
        print "Speed Limit", self.speed_limit
        self.speed_limit = self.speed_limit * 1000 / 3600. # m/s
        # TODO: Add other member variables you need below
        self.lightindx = -0x7FFFFFFF
        self.stop = False
        self.isYellow = False
        self.current_linear_velocity  = None
        self.current_angular_velocity = None
        self.lasta = 0.0
        self.frameno = 0
        # New
        self.stop_accel = 0.0
        self.stop_v = 0.0
        rospy.spin()

    def current_velocity_cb(self, msg):
        #rospy.loginfo('Current Velocity Received...')
        self.current_linear_velocity = msg.twist.linear.x
        self.current_angular_velocity = msg.twist.angular.x

    def pose_cb(self, msg):
        if self.wps == None: return
        # TODO: Implement
        #if self.count <  3:
        #    print(msg)
        #if (self.count % 256) == 0: Causes system to not work
        #    self.speed_limit =  rospy.get_param_cached('/waypoint_loader/velocity', 80.46)
        #print "Speed Limit", self.speed_limit
        #self.speed_limit = self.speed_limit * 1000 / 3600. # m/s
        self.count += 1
        # x, y, and z give the current position from the pose message
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        #print (self.wps[0])
        # if the previous waypoint of the car is unknown, find the best one
        if self.wp_index == None:
            mindist = 1000000.0
            bestind = None
            for i in range(len(self.wps)):
                xw = self.wps[i].pose.pose.position.x
                yw = self.wps[i].pose.pose.position.y
                zw = self.wps[i].pose.pose.position.z
                dist = math.sqrt((x-xw)**2+(y-yw)**2+(z-zw**2))
                if dist < mindist:
                    bestind = i
                    mindist = dist
            self.wp_index = bestind
        # Otherwise, increment the index to find the closest waypoint
        else:
            bestind = self.wp_index
            i = bestind
            xw = self.wps[i].pose.pose.position.x
            yw = self.wps[i].pose.pose.position.y
            zw = self.wps[i].pose.pose.position.z
            mindist = math.sqrt((x-xw)**2+(y-yw)**2+(z-zw**2)) 
            while True:
                i += 1
                if i >= len(self.wps): i = 0
                xw = self.wps[i].pose.pose.position.x
                yw = self.wps[i].pose.pose.position.y
                zw = self.wps[i].pose.pose.position.z
                dist = math.sqrt((x-xw)**2+(y-yw)**2+(z-zw**2))
                if dist > mindist: break
                mindist = dist
                bestind = i
        # Make sure that the closest waypoint selected is in front of the car, not behind it!
        # Calculate the changes in x and y from the closest waypoint to the waypoint 10 ahead
        index10 = bestind + 10 
        if index10  >=  len(self.wps): index10 -= len(self.wps)
        dx = self.wps[index10].pose.pose.position.x - \
             self.wps[bestind].pose.pose.position.x
        dy = self.wps[index10].pose.pose.position.y - \
             self.wps[bestind].pose.pose.position.y
        # If the change in x is greater, compare best waypoint x with the position x
        # Increment bestind until the sign of the change in x to advance 10 waypoints
        # matches the sign of the change in x to move from the position x to the best wp 
        if abs(dx) > abs(dy):
            if dx > 0.0: sign = 1
            else: sign = -1
            while True:
                dx = self.wps[bestind].pose.pose.position.x - x
                if dx > 0.0: signb = 1
                else: signb = -1
                if sign == signb: break
                bestind += 1
                if bestind >= len(self.wps): bestind = 0
        # If the change in y is greater, compare best waypoint y with the position y
        # Increment bestind until the sign of the changes in y to advance 10 waypoints
        # matches the sign of the change in y to move from the position y to the best wp 
        else:
            if dy > 0.0: sign = 1
            else: sign = -1
            while True:
                dy = self.wps[bestind].pose.pose.position.y - y
                if dy > 0.0: signb = 1
                else: signb = -1
                if sign == signb: break
                bestind += 1
                if bestind >= len(self.wps): bestind = 0
        self.wp_index = bestind
        if (self.count < 3):
            print("Best waypoint", bestind, self.wps[bestind].pose.pose.position.x, \
                                            self.wps[bestind].pose.pose.position.y, \
                                            self.wps[bestind].pose.pose.position.z)
        # Update the velocities of the waypoints from the bestind to bestind + LOOKAHEAD_WPS - 1
        # for now, use 10.0 m/s (about 22 MPH)
        i = bestind
        if self.current_linear_velocity != None:
            lastspeed = self.current_linear_velocity
        else:
            lastspeed =  self.wps[bestind].twist.twist.linear.x
        # Calculate acceleration (in the reverse direction) to stop the car
        # before the next red (or yellow?) traffic light
 
        if (self.count & 0x3f) == 0: print "lightindx", self.lightindx, self.wp_index, \
                                                        self.lasta
        if self.lightindx > 0:
            stoptarget = self.lightindx - 5
            if stoptarget < 0: stoptarget += len(self.wps)
            xw = self.wps[i].pose.pose.position.x
            yw = self.wps[i].pose.pose.position.y
 
            dxl = self.wps[stoptarget].pose.pose.position.x-xw
            dyl = self.wps[stoptarget].pose.pose.position.y-yw
            s = math.sqrt(dxl*dxl + dyl*dyl)
            #Calculate difference i - light waypoint with wraparound
            di = i - stoptarget
            if di < 0: di += len(self.wps)
            #if (s < .1) or ((di > 20) and self.stop): a = 5.0
            if (s < 0.01):  a = 9.0
            else:
                a = lastspeed * lastspeed / (s + s) #(s + s) + 1.5
                if a > 9.0: a = 9.0
        else: a = 0.0
        count  = LOOKAHEAD_WPS
        finalwps = []
        v = lastspeed
        nexti = i + 1
        if nexti >= len(self.wps): nexti = 0
        if (a > 4.0) and not(self.isYellow and a >= 6.0):
            #print "Stopping", i
            if not self.stop:
                self.stop_accel = a
                self.stop_v = v
            self.stop = True
        if self.stop:
            self.stop_v -= 0.02 * self.stop_accel
            if self.stop_v < 0.0: self.stop_v = 0.0 
        #print "a, v", a, v
        while True:
            if not self.stop:
                self.wps[i].twist.twist.linear.x = self.speed_limit #22.352
            else:
                xw = self.wps[i].pose.pose.position.x
                yw = self.wps[i].pose.pose.position.y
                dx = self.wps[nexti].pose.pose.position.x-xw
                dx = self.wps[nexti].pose.pose.position.y-yw
                d = math.sqrt(dx * dx + dy * dy)
                if v > .01: v -= (a + 1.0) * d/v
                else: v = 0.0
                if v < 0.0: v = 0.0
                if (i >= stoptarget) and (i <= self.lightindx): v = 0.0
                if self.stop_v < v: v = self.stop_v
                self.wps[i].twist.twist.linear.x = v
            finalwps += [self.wps[i]]
            count -= 1
            if count == 0: break
            i += 1
            if i == len(self.wps): i = 0
        # Publish
        self.lasta = a
        lane = Lane()
        lane.header.frame_id='/final'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = finalwps
        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, lane):
        # TODO: Implement
        print ("Waypoints  message received")
        #print(lane.waypoints[0:10])
        self.wps = lane.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if msg.data != -0x7FFFFFFF:
            if msg.data < 0:
                self.lightindx = -msg.data
                self.isYellow  = True
            else:
                self.isYellow  = False
                self.lightindx = msg.data
        else:
            self.stop = False
            self.lightindx = -0x7FFFFFFF
        #self.lightindx -= 40
        #if self.lightindx < 0: self.lightindx += len(self.wps)
        #print "Traffic callback msg", msg


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    #def lights_cb(self, msg):
    #    print "Traffic lights", msg

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
