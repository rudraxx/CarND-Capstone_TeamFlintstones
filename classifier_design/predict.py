"""
This script makes the prediction step


Author: @jjaviergalvez

References: 
[1] www.codesofinterest.com/2017/08/bottleneck-features-multi-class-classification-keras.html
[2] https://blog.keras.io/building-powerful-image-classification-models-using-very-little-data.html


"""

import os.path
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

from keras.applications.mobilenet import MobileNet
from keras.preprocessing import image
from keras.applications.mobilenet import preprocess_input
from keras.models import load_model
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

# load the MobileNet model 
model_features = MobileNet(input_shape=(IMG_WIDTH, IMG_HEIGHT, 3),
                               include_top=False, 
                               weights='imagenet')

img_path = 'test.png'
img = image.load_img(img_path, target_size=(224, 224))
x = image.img_to_array(img)
x = np.expand_dims(x, axis=0)
x = preprocess_input(x)

# get the bottleneck features from the mobilenet
bottleneck_features = model_features.predict(x)

# load our classfier model
model_classification = load_model(TOP_MODEL_PATH)

# use the bottleneck features to get the final classification
class_predicted = model_classification.predict(bottleneck_features)

#print('Predicted:', decode_predictions(preds, top=3)[0])
preds = decode_lights_predictions(class_predicted)

print(preds)

"""
orig = cv2.imread(img_path)

# display the predictions with the image
cv2.putText(orig, "{}".format(preds), (10, 30),
            cv2.FONT_HERSHEY_PLAIN, 1.5, (43, 99, 255), 2)

cv2.imshow("Classification", orig)
cv2.waitKey(0)
cv2.destroyAllWindows()
"""