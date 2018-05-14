import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np


class TLClassifier(object):
    def __init__(self, is_sim):
        
        if is_sim:
            #PATH_TO_MODEL = 'light_classification/models/rfcn_resnet101_coco_sim/frozen_inference_graph.pb'
            PATH_TO_MODEL = r'light_classification/models/ssd_inception_v2_coco_sim/frozen_inference_graph.pb'
        else:
            PATH_TO_MODEL = r'light_classification/models/rfcn_resnet101_coco_byrd/frozen_inference_graph.pb'
            #PATH_TO_MODEL = r'light_classification/models/ssd_inception_v2_coco_byrd/frozen_inference_graph.pb'
            
        self.graph = tf.Graph()

        with self.graph.as_default():

            with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as f:
                graph_def = tf.GraphDef()
                graph_def.ParseFromString(f.read())
                tf.import_graph_def(graph_def, name='')
        
            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

        self.sess = tf.Session(graph=self.graph)



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
        #print("scores: ",scores[0], ", classes: ", classes[0])

        if scores[0] > 0.5:
            if classes[0] == 1:
                return TrafficLight.GREEN
            elif classes[0] == 2:
                return TrafficLight.RED
            elif classes[0] == 3:
                return TrafficLight.YELLOW
        
        return TrafficLight.UNKNOWN
