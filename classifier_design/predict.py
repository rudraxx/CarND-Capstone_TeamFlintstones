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
from keras.applications.mobilenet import preprocess_input, decode_predictions
from keras.models import load_model
import numpy as np

import cv2

from consts import *


# load the class_indices saved in the earlier step
class_dictionary = np.load('class_indices.npy').item()

# load the MobileNet model 
model_features = MobileNet(input_shape=(IMG_WIDTH, IMG_HEIGHT, 3),
                               include_top=False, 
                               weights='imagenet')

img_path = 'test.jpg'
img = image.load_img(img_path, target_size=(224, 224))
x = image.img_to_array(img)
x = np.expand_dims(x, axis=0)
x = preprocess_input(x)

# get the bottleneck features from the mobilenet
bottleneck_features = model_features.predict(x)

# load our classfier model
model_classification = load_model(TOP_MODEL_PATH)

# use the bottleneck features to get the final classification
class_predicted = model_classification.predict_classes(bottleneck_features)

# this lines of code are credit from [1] 
inID = class_predicted[0]
inv_map = {v: k for k, v in class_dictionary.items()}
label = inv_map[inID]

# get the prediction label
print("Image ID: {}, Label: {}".format(inID, label))


orig = cv2.imread(img_path)

# display the predictions with the image
cv2.putText(orig, "Predicted: {}".format(label), (10, 30),
            cv2.FONT_HERSHEY_PLAIN, 1.5, (43, 99, 255), 2)

cv2.imshow("Classification", orig)
cv2.waitKey(0)
cv2.destroyAllWindows()
