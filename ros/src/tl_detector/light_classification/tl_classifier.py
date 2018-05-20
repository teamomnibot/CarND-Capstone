from styx_msgs.msg import TrafficLight
import tensorflow as tf

class TLClassifier(object):
    def __init__(self, sim_true):
        #TODO load classifier
        if sim_true:
            GRAPH_PATH = r'light_classification/model/inf_graph_sim.pb'
        else:
            GRAPH_PATH = r'light_classification/model/inf_graph_udacity.pb'
        self.graph = tf.Graph()
        
        
        with graph.as_default():
            new_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
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
            start = datetime.datetime.now()
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections], feed_dict={self.image_tensor: img_expand})
            end = datetime.datetime.now()
            duration = end - start
            print(duration.total_seconds())
        
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        
        if(scores[0] > self.thresh):
            if(classes[0] == 1):
                return TrafficLight.GREEN
            elif(classes[0] == 2):
                return TrafficLight.RED
            elif (classes[0] == 3):
                return TrafficLight.YELLOW
        else:
            return TrafficLight.UNKNOWN
