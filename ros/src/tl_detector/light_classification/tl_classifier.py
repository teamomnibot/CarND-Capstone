from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf
from datetime import datetime as dt
import rospy

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

        sim_true                 = rospy.get_param('/sim_true')

        self.thresh = 0.65

        if sim_true:
            GRAPH_PATH = r'light_classification/model/inf_graph_sim.pb'
        else:
            GRAPH_PATH = r'light_classification/model/inf_graph_udacity.pb'
        rospy.logwarn("using %s"%GRAPH_PATH)

        #------------ Uncomment these one at a time to test each of the models ---------------#
        #             We need to see how well they each detect traffic lights                 #

        #GRAPH_PATH = r'light_classification/model/frozen_inference_22175.pb'
        #GRAPH_PATH = r'light_classification/model/frozen_inference_24330.pb'
        #GRAPH_PATH = r'light_classification/model/frozen_inference_26484.pb'
        #GRAPH_PATH = r'light_classification/model/frozen_inference_28639.pb'

        self.graph = tf.Graph()
        
        
        with self.graph.as_default():
            new_graph_def = tf.GraphDef()
            with tf.gfile.GFile(GRAPH_PATH, 'rb') as fid:
                serialized_graph = fid.read()
                new_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(new_graph_def, name='')
            
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
        #TODO implement light color prediction
        #  Check what the styx_msgs/TrafficLight msg should look like to 
        #  create the correct output
        
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            start = dt.now()
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections], feed_dict={self.image_tensor: img_expand})
            end = dt.now()
            duration = end - start
            #print "duration of image classification: ", duration.total_seconds()
        
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        print scores 
        
        if(scores[0] > self.thresh):
            if(classes[0] == 1):
                return TrafficLight.GREEN
            elif(classes[0] == 2):
                return TrafficLight.RED
            elif (classes[0] == 3):
                return TrafficLight.YELLOW
        else:
            return TrafficLight.UNKNOWN
