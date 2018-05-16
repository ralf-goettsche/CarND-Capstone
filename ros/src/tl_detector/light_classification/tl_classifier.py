import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime
import cv2


class TLClassifier(object):
    def __init__(self, is_sim):
        
        if is_sim:
            #PATH_TO_MODEL = 'light_classification/models/rfcn_resnet101_coco_sim/frozen_inference_graph.pb'
            #PATH_TO_MODEL = r'light_classification/models/ssd_inception_v2_coco_sim/frozen_inference_graph.pb'
            #PATH_TO_MODEL = r'light_classification/models/ssd_inception_v2_coco_150_150_sim/frozen_inference_graph.pb'
            PATH_TO_MODEL = r'light_classification/models/ssd_inception_v2_coco_300_300_sim/frozen_inference_graph.pb'
        else:
            #PATH_TO_MODEL = r'light_classification/models/rfcn_resnet101_coco_byrd/frozen_inference_graph.pb'
            PATH_TO_MODEL = r'light_classification/models/ssd_inception_v2_coco_byrd/frozen_inference_graph.pb'
            
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


    def integrate_scores(self,scores,classes):
        tmp = {1:0,2:0,3:0,4:0}    
	total = 0
        for s,c in zip(scores,classes):
            tmp[c]+=s
	    total+=s
        mx = 0.5
        sel = 4
        for key,val in tmp.items():
            if val/total>mx:
               sel= key
               mx= val/total
        return (sel,mx)
    
    

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        image_size = cv2.resize(image,(150,150))
        image_exp = np.expand_dims(image_size, axis=0)
        #image_exp = np.expand_dims(image, axis=0)
        start = datetime.datetime.now()
        (boxes, scores, classes, num_detection) = self.sess.run([self.boxes, self.scores, self.classes, self.num_detections], feed_dict={self.image_tensor: image_exp})
        end = datetime.datetime.now()
        c = end - start
        #print("Time for classification: ", c.total_seconds())

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        #rospy.loginfo("scores: %f, classes: %d", scores[0], classes[0])
        #rospy.loginfo("scores: %s, classes: %s", str(scores), str(classes))
        #print("scores: ",scores[0], ", classes: ", classes[0])

        out = self.integrate_scores(scores,classes)
        #rospy.loginfo("scores: %f, classes: %d",out[1] , out[0])
        #civ2.imwrite('/home/student/output/'+str(start)+'_{0}_{1:.2f}'.format(out[0],out[1])+'.png',image_size)

        if out[1] > 0.5:
            if out[0] == 1:
                return TrafficLight.GREEN
            elif out[0] == 2:
                return TrafficLight.RED
            elif out[0] == 3:
                return TrafficLight.YELLOW
        #if scores[0] > 0.5:
        #    if classes[0] == 1:
        #        return TrafficLight.GREEN
        #    elif classes[0] == 2:
        #        return TrafficLight.RED
        #    elif classes[0] == 3:
        #        return TrafficLight.YELLOW
        return TrafficLight.UNKNOWN
