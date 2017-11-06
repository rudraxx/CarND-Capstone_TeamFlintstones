from styx_msgs.msg import TrafficLight

from keras.models import load_model
from keras.utils.generic_utils import CustomObjectScope
from keras.applications.mobilenet import relu6, DepthwiseConv2D, preprocess_input
from keras.preprocessing import image
import tensorflow as tf

import cv2
import os
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

        # directory of this file where the class is defined
        self.folder_path = os.path.dirname(os.path.abspath(__file__))

        with CustomObjectScope({'relu6': relu6,'DepthwiseConv2D': DepthwiseConv2D}):
    		self.model = load_model(self.folder_path + '/capstone_model.h5')
    		# this is need to done for sync systems as is sugested here: https://github.com/fchollet/keras/issues/2397
	        self.model._make_predict_function()
	        self.graph = tf.get_default_graph()
    	

    def decode_lights_predictions(self, preds, top=3):
	    """Decodes the prediction.

	    """
	    # load the class_indices saved in the earlier step
	    class_index = np.load(self.folder_path + '/class_indices.npy').item() 
	    #invert the index
	    class_index = {v: k for k, v in class_index.items()}

	    pred = preds[0]
	    top_indices = pred.argsort()[-top:][::-1]
	    result = []
	    for i in top_indices:
	    	result.append((class_index[i], pred[i]))
	    result.sort(key=lambda x: x[1], reverse=True)

	    return result


    def get_classification(self, img):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        # Javi note: "image" is not the image input, is a set of tools imports
        # from keras.preprocessing
        
        # Prepare the image

        # INTER_NEAREST as it the same it was used on training
        img = cv2.resize(img,(224, 224), cv2.INTER_NEAREST)
        x = image.img_to_array(img)
        x = np.expand_dims(x, axis=0)
        x = preprocess_input(x)
        
        # run inside a graph as is sugested here: https://github.com/fchollet/keras/issues/2397
        with self.graph.as_default():
        	# get the probabilities for each class
        	preds = self.model.predict(x)
        	
	        # map and ordering the predictions
	        preds = self.decode_lights_predictions(preds)

	        # as is in order, the first element is the highest certainaty
	        label_pred = preds[0][0]

	        # TODO: maybe add some threshold to the probabilities to only meke
	        # a predictions when we are very shure about the label
	        if label_pred == "red":
	        	return TrafficLight.RED
	        if label_pred == "green":
	        	return TrafficLight.GREEN
	        if label_pred == "no":
	        	return TrafficLight.UNKNOWN
