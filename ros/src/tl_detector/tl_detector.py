#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.tl_dict = {}
        self.count  = 0
        self.count1 = 0
        self.wp_index = None
        self.trace = False
        #rospy.spin()
	self.loop()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        print "Base waypoints received by tl_detector"

    def traffic_cb(self, msg):
        self.lights = msg.lights
        if self.waypoints == None: 
            if (self.count & 0xff) == 0: print "No waypoints yet"
            self.count += 1
            return
        # Create/maintain a dictionary of traffic lights
        for i in range (0, len(self.lights)):
            xl = self.lights[i].pose.pose.position.x
            yl = self.lights[i].pose.pose.position.y
            state = self.lights[i].state
            if (xl, yl) in self.tl_dict:
                oldstate, wpl = self.tl_dict[(xl, yl)]
                self.tl_dict[(xl, yl)] =  (state, wpl)
            else:
                # Find the closest waypoint to the light's stop line
                mindist = 1000000.0
                for p in self.stop_line_positions:
                    xs = p[0]
                    ys = p[1]
                    dx = xs - xl
                    dy = ys - yl
                    d = math.sqrt(dx * dx + dy * dy)
                    if d < mindist:
                        mindist = d
                        bestp = p
                xls = bestp[0]
                yls = bestp[1]
                mindist = 1000000.0
                for j in range(0, len(self.waypoints)):
                    xw = self.waypoints[j].pose.pose.position.x
                    yw = self.waypoints[j].pose.pose.position.y
                    dx = xls - xw
                    dy = yls - yw
                    dist = math.sqrt(dx * dx + dy * dy)
                    if dist < mindist: 
                       mindist = dist
                       bestind = j
                self.tl_dict[(xl, yl)] = (state, bestind)
        #if (self.count & 0xff) == 0:
        #   print "TL dictionary"
        #   print(self.tl_dict)
        self.count += 1

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        #self.camera_image = msg

    def loop(self): 
      rate = rospy.Rate(5) # 50Hz
      while not rospy.is_shutdown():     
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights and yellow at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            if    state == TrafficLight.RED:
                light_wp = light_wp
            elif state == TrafficLight.YELLOW:
                light_wp = -light_wp
            #elif state == TrafficLight.GREEN:
	     #   light_wp = light_wp
	        #print "GREEN LIGHT!!!" 
            else:light_wp = -0x7FFFFFFF
            #light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            #print "Publishing", light_wp
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            #print "Publishing", self.last_wp
        self.state_count += 1
        
        rate.sleep()

	

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if self.waypoints == None: return 0
        # if the previous waypoint of the car is unknown, find the best one
        # x, y, and z give the current position from the pose message
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        if self.wp_index == None:
            mindist = 1000000.0
            bestind = None
            for i in range(len(self.waypoints)):
                xw = self.waypoints[i].pose.pose.position.x
                yw = self.waypoints[i].pose.pose.position.y
                zw = self.waypoints[i].pose.pose.position.z
                dist = math.sqrt((x-xw)**2+(y-yw)**2+(z-zw**2))
                if dist < mindist:
                    bestind = i
                    mindist = dist
            self.wp_index = bestind
        # Otherwise, increment the index to find the closest waypoint
        else:
            bestind = self.wp_index
            i = bestind
            xw = self.waypoints[i].pose.pose.position.x
            yw = self.waypoints[i].pose.pose.position.y
            zw = self.waypoints[i].pose.pose.position.z
            mindist = math.sqrt((x-xw)**2+(y-yw)**2+(z-zw**2)) 
            while True:
                i += 1
                if i >= len(self.waypoints): i = 0
                xw = self.waypoints[i].pose.pose.position.x
                yw = self.waypoints[i].pose.pose.position.y
                zw = self.waypoints[i].pose.pose.position.z
                dist = math.sqrt((x-xw)**2+(y-yw)**2+(z-zw**2))
                if dist > mindist: break
                mindist = dist
                bestind = i
            self.wp_index = bestind
        return bestind

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            #if car_position > 5500: self.trace = True

            #print "Getting closest waypoint to car", car_position

            #TODO find the closest visible traffic light (if one exists)
            mindist = 1000000
            if self.trace: print "Find closest light", car_position
            for i in self.tl_dict:
                j = self.tl_dict[i][1]
                d = j - car_position
                if d < 0: d += len(self.waypoints)
                if d < mindist:
                    bestind = j
                    light = i
                    mindist = d
                if self.trace: print car_position, j, d, mindist, bestind
            if light: state = self.tl_dict[light][0]
            #if (self.count1 & 0x7f) == 0: print "Car wp index", car_position, bestind, state
            self.count1 += 1
        if light:
	    #print("found the closest light!")	
            #  state = self.get_light_state(light)
            light_wp = bestind
            return light_wp, state
        #self.waypoints = None
        #else: print("not found the closest light!")
	return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
