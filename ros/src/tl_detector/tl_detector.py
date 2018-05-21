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
import yaml
from math import sqrt
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):

        # Exchange comment will let you see the estimated traffic light color and processing time 
        rospy.init_node('tl_detector')
        #rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # Define range in which traffic lights can be seen
        self.range = 100

        # Stores accumulated processing time of image callback
        self.image_cb_total_time = 0.0

        # Stores how often image callback has been called
        self.image_cb_counter = 0

        # Boolean if image is currently available
        self.has_image = False

        self.loop()
        
    def loop(self):        
        # loop implemented rather than spin to control the frequency precisely
        rate = rospy.Rate(5)
        
        while not rospy.is_shutdown():
            # if pose and base_waypoints are filled
            if self.has_image:
                self.process_image()
            rate.sleep()

    # Stores subscribed current_pose of car
    def pose_cb(self, msg):
        self.pose = msg

    # Stores subscribed base_waypoints of car
    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
        self.waypoint_tree = KDTree(waypoints_2d)

    # Stores subscribed list of traffic lights tha are in the map
    def traffic_cb(self, msg):
        self.lights = msg.lights

    # Stores sensor image
    def image_cb(self, msg):
        self.has_image = True
        self.camera_image = msg

    # Process sensor image
    def process_image(self):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        # Store current time
        t1 = rospy.get_time()
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

        # Store current time
        t2 = rospy.get_time()

        # Print average processing time for this callback
        self.image_cb_total_time += (t2 - t1)
        self.image_cb_counter += 1
        self.has_image = False
        rospy.logdebug("TL Detector Current proc. time : %f s", (t2 - t1))
        rospy.logdebug("TL Detector Average proc. time : %f s", self.image_cb_total_time / self.image_cb_counter)

    def get_closest_waypoint(self, x, y):
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

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
        classified_state = self.light_classifier.get_classification(cv_image)
        rospy.logdebug("Light state %s, classified state: %s", light.state, classified_state)
        return classified_state

    def get_distance(self, x1, y1, x2, y2):
        return sqrt( (x1-x2)**2 + (y1-y2)**2 )

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        line_wp_idx = -1

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):

            # Get waypoint index of the car
            x_car = self.pose.pose.position.x
            y_car = self.pose.pose.position.y
            car_wp_idx = self.get_closest_waypoint(x_car, y_car)

            # Set maximum front distance to next traffic light
            if self.waypoints:
                max_dist = min(len(self.waypoints.waypoints), self.range)
            else:
                max_dist = self.range

            # Loop over list of traffic lights to check distance to the car
            for i, traffic_light in enumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx

                if 0 <= d < max_dist:
                    max_dist = d
                    light = traffic_light
                    line_wp_idx = temp_wp_idx

        if light:
            
            # Get light status
            state = self.get_light_state(light)
            #print("Traffic light found:", car_wp_idx, line_wp_idx, state)
            return line_wp_idx, state

        rospy.logdebug("No traffic light within sensor image")
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
