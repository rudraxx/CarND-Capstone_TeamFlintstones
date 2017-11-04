"""
This script makes the prediction step


Author: @jjaviergalvez

References: 
[1] www.codesofinterest.com/2017/08/bottleneck-features-multi-class-classification-keras.html
[2] https://blog.keras.io/building-powerful-image-classification-models-using-very-little-data.html


"""

import os.path
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

from keras.preprocessing import image
from keras.models import load_model
from keras.utils.generic_utils import CustomObjectScope
from keras.applications.mobilenet import relu6, DepthwiseConv2D, preprocess_input
import numpy as np

import cv2

from consts import *


def decode_lights_predictions(preds, top=3):
    """Decodes the prediction.

    """
    
    # load the class_indices saved in the earlier step
    class_index = np.load('class_indices.npy').item() 
    #invert the index
    class_index = {v: k for k, v in class_index.items()}

    pred = preds[0]   
    top_indices = pred.argsort()[-top:][::-1]
    result = []
    for i in top_indices:
    	result.append((class_index[i], pred[i]))    
    result.sort(key=lambda x: x[1], reverse=True)
        
    return result


img_path = 'test.png'
img = image.load_img(img_path, target_size=(224, 224))
x = image.img_to_array(img)
x = np.expand_dims(x, axis=0)
x = preprocess_input(x)

# This comes from: https://github.com/fchollet/keras/issues/7431
with CustomObjectScope({'relu6': relu6,'DepthwiseConv2D': DepthwiseConv2D}):
    model = load_model(CAPSTONE_MODEL_PATH)


# use the bottleneck features to get the final classification
class_predicted = model.predict(x)
print(class_predicted)

#print('Predicted:', decode_predictions(preds, top=3)[0])
preds = decode_lights_predictions(class_predicted)

print(preds)
print(preds[0][0])
print(preds[0][1])
"""
orig = cv2.imread(img_path)

# display the predictions with the image
cv2.putText(orig, "{}".format(preds), (10, 30),
            cv2.FONT_HERSHEY_PLAIN, 1.5, (43, 99, 255), 2)

cv2.imshow("Classification", orig)
cv2.waitKey(0)
cv2.destroyAllWindows()
"""