from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
import rospy

class TLClassifier(object):
    def __init__(self):

        # Define minimum and maximum hsv threshold values for red, yellow and green
        self.lower_red = np.array([0,120,150])
        self.upper_red = np.array([25,255,255])
        self.lower_yel = np.array([30,150,150])
        self.upper_yel = np.array([50,255,255])
        self.lower_gre = np.array([60,150,150])
        self.upper_gre = np.array([90,255,255])

        # Change between classification methods
        # True: Check color AND shape to match with a traffic light (Slower)
        # False: Simple count of colors (Faster [~40%])
        self.check_shape = False
        rospy.logdebug("Check traffic light shapes: %s", str(self.check_shape))

        # Define the minimum number of white pixels for the simple count classification
        self.min_num_white_pixs = 100

        # Define minimum and maximum traffic light size in pixels for shape classification
        self.min_side = 5
        self.max_side = 45

    def check_connected_cluster(self, thresh_image):

        # Connected component clustering
        connectivity = 4
        output = cv2.connectedComponentsWithStats(thresh_image, 
            connectivity, cv2.CV_32S)
        num_labels = output[0]
        stats = output[2]

        # Run through clusters and check if cluster is within traffic light size
        for i in range(num_labels):

            # Ignore background
            if i == 0:
                continue

            # Get width and height of cluster
            width = stats[i][2]
            height = stats[i][3]

            # If cluster has the shape of a traffic light mark it
            if(width > self.min_side and width < self.max_side and
                height > self.min_side and height < self.max_side):
                
                return True

        return False

    def check_for_colored_traffic_light_shapes(self, thresh_red, thresh_yel, thresh_gre):
        """Determines if a properly sized traffic light has been found 
        in thresholded image
        """

        has_red_lights = self.check_connected_cluster(thresh_red)
        has_yel_lights = self.check_connected_cluster(thresh_yel)
        has_gre_lights = self.check_connected_cluster(thresh_gre)

        return has_red_lights, has_yel_lights, has_gre_lights

    def check_for_colors(self, thresh_red, thresh_yel, thresh_gre):
        """Determines if tresholded colored has more than color_thres hits
        """

        # Count white pixels
        count_red = cv2.countNonZero(thresh_red)
        count_yel = cv2.countNonZero(thresh_yel)
        count_gre = cv2.countNonZero(thresh_gre)
        
        # Find traffic light clusters within this thresholded colorspace
        has_red_lights = (count_red > self.min_num_white_pixs)
        has_yel_lights = (count_yel > self.min_num_white_pixs)
        has_gre_lights = (count_gre > self.min_num_white_pixs)

        return has_red_lights, has_yel_lights, has_gre_lights

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get mask each color
        mask_red = cv2.inRange(hsv, self.lower_red, self.upper_red)
        mask_yel = cv2.inRange(hsv, self.lower_yel, self.upper_yel)
        mask_gre = cv2.inRange(hsv, self.lower_gre, self.upper_gre)

        # Change between classification methods
        # True: Also check shape of colors to match a traffic light (Slower)
        # False: Simple count of colors (Faster)
        if(self.check_shape):
            has_red_lights, has_yel_lights, has_gre_lights = \
                self.check_for_colored_traffic_light_shapes(mask_red, mask_yel, mask_gre)
        else:
            has_red_lights, has_yel_lights, has_gre_lights = \
                self.check_for_colors(mask_red, mask_yel, mask_gre)

        # Return the current traffic light state
        if(has_red_lights and not has_yel_lights and not has_gre_lights):
            rospy.loginfo("Traffic Light is: RED")
            return TrafficLight.RED
        elif(not has_red_lights and has_yel_lights and not has_gre_lights):
            rospy.loginfo("Traffic Light is: YELLOW")
            return TrafficLight.YELLOW
        elif(not has_red_lights and not has_yel_lights and has_gre_lights):
            rospy.loginfo("Traffic Light is: GREEN")
            return TrafficLight.GREEN
        else:
            rospy.loginfo("Bad angle to traffic light")
            return TrafficLight.UNKNOWN