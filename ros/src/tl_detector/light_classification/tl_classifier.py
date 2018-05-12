from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np


class TLClassifier(object):
    def __init__(self,is_sim):

        if is_sim:
            PATH_TO_TRFLIGHT_MODEl = 'light_classification/models/rfcn_resnet101_coco_sim/frozen_inference_graph.pb'
        else:
            PATH_TO_TRFLIGHT_MODEL = 'light_classification/model/rfcn_resnet101_coco_byrd/frozen_inference_graph.pb'


        self.sess = tf.Session()

    
        tf.saved_model.loader.load(self.sess, ['serve'], PATH_TO_TRFLIGHT_MODEL)
        self.graph = tf.Graph()
        self.graph = tf.get_default_graph()

        self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
        self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        self.scores = self.graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.graph.get_tensor_by_name('num_detections:0')


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        image_exp = np.expand_dims(image, axis=0)
        (boxes, scores, classes, num_detection) = self.sess.run([self.boxes, self.scores, self.classes, self.num_detections], feed_dict={self.image_tensor: image_exp})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        rospy.logdebug("scores: %d, classes: %d", scores[0], classes[0])
        print("scores: %d, classes: %d", scores[0], classes[0])

        if scores[0] > 0.5:
            if classes[0] == 1:
                return TrafficLight.GREEN
            elif classes[0] == 2:
                return TrafficLight.RED
            elif classes[0] == 3:
                return TrafficLight.YELLOW
        
        return TrafficLight.UNKNOWN
