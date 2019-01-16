from styx_msgs.msg import TrafficLight

import rospy
import tensorflow as tf
import numpy as np
import time


class TLClassifier(object):

    def __init__(self, model_file):
        rospy.loginfo("Loading model file: %s", model_file)
        self.detection_graph = self.load_graph(model_file)
        self.session = tf.Session(graph=self.detection_graph)
        rospy.loginfo("Model loaded")

        self.labelmap = {1: TrafficLight.GREEN,
                         2: TrafficLight.RED,
                         3: TrafficLight.YELLOW,
                         4: TrafficLight.UNKNOWN}

    def load_graph(self, model_file):
        detection_graph = tf.Graph()

        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(model_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        return detection_graph

    def load_image_into_numpy_array(self, image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        rospy.loginfo("Starting detection")
        detection_graph = self.detection_graph
        # with detection_graph.as_default():
        #     with tf.Session(graph=detection_graph) as sess:
        # Definite input and output Tensors for detection_graph
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        image_np = image
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)

        time0 = time.time()

        # Actual detection.
        (boxes, scores, classes, num) = self.session.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        time1 = time.time()

        rospy.loginfo("Detection in {}ms".format((time1 - time0) * 1000))
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        min_score_thresh = .50
        combined = sorted([x for x in zip(classes, scores) if x[1] > min_score_thresh], key=lambda tup: tup[1])
        rospy.loginfo("Sorted scores: %s", combined)
        return self.labelmap[combined[-1][0]] if len(combined) > 1 else TrafficLight.UNKNOWN
