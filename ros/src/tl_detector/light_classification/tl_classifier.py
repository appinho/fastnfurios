from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):

        # Define minimum and maximum traffic light size in pixels
        self.min_side = 5
        self.max_side = 45

        # Define minimum and maximum hsv values for red, yellow and green
        self.lower_red = np.array([0,150,150])
        self.upper_red = np.array([20,255,255])
        self.lower_yel = np.array([30,150,150])
        self.upper_yel = np.array([50,255,255])
        self.lower_gre = np.array([60,150,150])
        self.upper_gre = np.array([90,255,255])

    def check_for_colored_clusters(self, thresh_image):
        """Determines if a properly sized traffic light has been found 
        in thresholded image

        Args:
            image (cv::Mat): thresh_image containing 
            the binarized color specification (red/green)

        Returns:
            bool: If this color has been found in thresholded image
        """

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

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only each color
        mask_red = cv2.inRange(hsv, self.lower_red, self.upper_red)
        mask_yel = cv2.inRange(hsv, self.lower_yel, self.upper_yel)
        mask_gre = cv2.inRange(hsv, self.lower_gre, self.upper_gre)

        # Find traffic light clusters within this thresholded colorspace
        has_red_lights = self.check_for_colored_clusters(mask_red)
        has_yel_lights = self.check_for_colored_clusters(mask_yel)
        has_gre_lights = self.check_for_colored_clusters(mask_gre)

        #print(has_red_lights, has_yel_lights ,has_gre_lights)

        # Return the current traffic light state
        if(has_red_lights and not has_yel_lights and not has_gre_lights):
            print("Red")
            return TrafficLight.RED
        elif(not has_red_lights and has_yel_lights and not has_gre_lights):
            print("Yellow")
            return TrafficLight.YELLOW
        elif(not has_red_lights and not has_yel_lights and has_gre_lights):
            print("Green")
            return TrafficLight.GREEN
        else:
            print("No traffic light")
            return TrafficLight.UNKNOWN