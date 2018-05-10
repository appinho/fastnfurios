from styx_msgs.msg import TrafficLight
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.counter = 0

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
        th = 200
        ret,th1 = cv2.threshold(red_channel_img,th,255,cv2.THRESH_BINARY)

        cv2.imwrite('test_images/image' + str(self.counter) + '.png', th1)
        self.counter += 1
        print(self.counter)

        return TrafficLight.UNKNOWN
