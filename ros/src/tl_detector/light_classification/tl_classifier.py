from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf
from datetime import datetime as dt
import rospy
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TLClassifier(object):
    def __init__(self):
        #load classifier

        sim_true                 = rospy.get_param('/sim_true')
        self.debug = False

        self.bridge = CvBridge()

        if self.debug==True:
            self.debug_image_pub = rospy.Publisher('/classified_image', Image, queue_size=1)

        self.thresh = 0.20
        working_dir = os.path.dirname(os.path.realpath(__file__))


        if sim_true:
            GRAPH_PATH = working_dir + '/model/frozen_inf_vatsal5kPlus2k.pb'
            #GRAPH_PATH = working_dir + '/model/frozen_inf_vatsal5k.pb'
            #GRAPH_PATH = working_dir + '/model/inf_graph_sim.pb'
        else:
            GRAPH_PATH = working_dir + '/model/inf_graph_udacity.pb'
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

        #rospy.logwarn(image.shape)
        
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
        #rospy.logwarn("scores and classes: ")
        #rospy.logwarn(scores[:3])
        #rospy.logwarn(classes[:3])

        h,w,d = image.shape
        #print h,w,d


        if self.debug==True:
            for k in range(len(boxes)):
                if scores[k]> 0.02:

                    y1 = int(boxes[k][0]*h)
                    x1 = int(boxes[k][1]*w)
                    y2 = int(boxes[k][2]*h)
                    x2 = int(boxes[k][3]*w)

                    c = (255,255,255)
                    if (classes[k] == 2): c = (0,0,255)
                    if (classes[k] == 1): c = (0,255,0)
                    if (classes[k] == 3): c = (0,255,255)


                    cv2.rectangle(image, (x1, y1), (x2, y2), (255,0,0), 2)
                    #t = str(classes[k])+"|"+str(int(scores[k]*100))
                    t = str(int(scores[k]*100))#+"%"
                    cv2.putText(image,t,(x1,y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5,c,2,cv2.LINE_AA)


            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))
        
        if(scores[0] > self.thresh):
            if(classes[0] == 1):
                return TrafficLight.GREEN
            elif(classes[0] == 2):
                return TrafficLight.RED
            elif (classes[0] == 3):
                return TrafficLight.YELLOW
        else:
            return TrafficLight.UNKNOWN
