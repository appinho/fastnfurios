from styx_msgs.msg import TrafficLight
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.counter = 0
        self.min_side = 10
        self.max_side = 45

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        # 3rd channel because image is stored BGR
        red_channel_img = image[:,:,2]

        # Threshold red channel
        th = 240
        ret, thresh = cv2.threshold(red_channel_img,th,255,cv2.THRESH_BINARY)

        # Connected component clustering
        connectivity = 8
        output = cv2.connectedComponentsWithStats(thresh, connectivity, cv2.CV_32S)
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

            # If cluster has the shape of a traffic light mark it as red light
            if(width > self.min_side and width < self.max_side and
                height > self.min_side and height < self.max_side):
                
                self.counter += 1
                #print("Red light!", stats[i], self.counter)
                return TrafficLight.RED

        # Debug code
        #cv2.imwrite('test_images/image' + str(self.counter) + '.png', image)
        #cv2.imwrite('test_images/bin_image' + str(self.counter) + '.png', thresh)
        #self.counter += 1

        return TrafficLight.UNKNOWN
