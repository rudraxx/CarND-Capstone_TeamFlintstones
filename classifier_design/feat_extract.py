"""
This scrpt made a feature extraction of the dataset using MobileNet and 
saving it to an *npy files as well the labels. Also it saves a 
dicctionary of the classes for future use in the prediction  step.


This is the directory structure that you sould have:

data/
    train/
	    red/
	        r001.jpg
	        r003.jpg
	        ...
	    green/
	        g001.jpg
	        g002.jpg
	        ...
	    no/
	        n001.jpg
	        n002.jpg
	        ...
    validation/
        red/
	        r001.jpg
	        r003.jpg
	        ...
	    green/
	        g001.jpg
	        g002.jpg
	        ...
	    no/
	        n001.jpg
	        n002.jpg
	        ...

Author: @jjaviergalvez

References: 
[1] www.codesofinterest.com/2017/08/bottleneck-features-multi-class-classification-keras.html
[2] https://blog.keras.io/building-powerful-image-classification-models-using-very-little-data.html

"""

# Hide anoying warning messages
import os.path
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

import numpy as np
from keras.preprocessing.image import ImageDataGenerator
from keras.applications.mobilenet import MobileNet
from keras.applications.mobilenet import preprocess_input

from consts import *


# build the MobileNet network without the final fully-connected layers with ImageNet weights
model = MobileNet(input_shape=(IMG_WIDTH, IMG_HEIGHT, 3),
							   include_top=False, 
							   weights='imagenet')
	
# Apply the same prerposessing used in MobileNet net
datagen = ImageDataGenerator(preprocessing_function=preprocess_input)

# data generator for training images
generator = datagen.flow_from_directory(TRAIN_DATA_DIR,  
										target_size=(IMG_WIDTH, IMG_HEIGHT),
										batch_size=1,
										class_mode=None,
										shuffle=False)

print('Extracting training features with MobileNet:')
bottleneck_features_train = model.predict_generator(generator, 
												steps=len(generator.filenames),
												verbose=1)

# Save the indices for reference in the prediction step
np.save('class_indices.npy', generator.class_indices)

np.save('bottleneck_features_train.npy', bottleneck_features_train)
np.save('train_labels.npy', generator.classes)

# data generator for validation images
generator = datagen.flow_from_directory(VALIDATION_DATA_DIR,
										target_size=(IMG_WIDTH, IMG_HEIGHT),
										batch_size=1,
										class_mode=None,
										shuffle=False)

print('Extracting validation features with MobileNet:')
bottleneck_features_validation = model.predict_generator(generator,
													steps=len(generator.filenames),
													verbose=1)

np.save('bottleneck_features_validation.npy', bottleneck_features_validation)
np.save('validation_labels.npy', generator.classes)
