"""
Train the clasifier


Author: @jjaviergalvez

References: 
[1] www.codesofinterest.com/2017/08/bottleneck-features-multi-class-classification-keras.html
[2] https://blog.keras.io/building-powerful-image-classification-models-using-very-little-data.html

"""
import os.path
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

import pickle

import numpy as np
from keras.models import Sequential
from keras.utils.np_utils import to_categorical
from keras.layers import Dropout, Flatten, Dense
from keras.callbacks import ModelCheckpoint, EarlyStopping

from consts import *

def load_bottleneck_data():
	"""
    Utility function to load bottleneck features.

    """
	# get the number of classes
	class_dictionary = np.load('class_indices.npy')
	num_classes = len(class_dictionary.item())

	# load bottleneck features saved earlier
	train_data = np.load('bottleneck_features_train.npy')

	# load labels saved earlier and convet them to categorical vectors
	train_labels = np.load('train_labels.npy')
	train_labels = to_categorical(train_labels, num_classes=num_classes)

	# load bottleneck features saved earlier
	validation_data = np.load('bottleneck_features_validation.npy')

	# load labels saved earlier and convet them to categorical vectors
	validation_labels = np.load('validation_labels.npy')
	validation_labels = to_categorical(validation_labels, num_classes=num_classes)

	return train_data, train_labels, validation_data, validation_labels, num_classes

def define_top_model(train_data, num_classes):
	"""
    Our top model (the classifier) goes here
    
    """
	model = Sequential()
	model.add(Flatten(input_shape=train_data.shape[1:]))
	model.add(Dense(256, activation='relu'))
	model.add(Dropout(0.5))
	model.add(Dense(num_classes, activation='softmax'))

	model.compile(optimizer='Adam',
	              loss='categorical_crossentropy', metrics=['accuracy'])
	return model


def train_top_model():
	# load bottleneck data
	train_data, train_labels, validation_data, validation_labels, num_classes = load_bottleneck_data()

	# get our top model
	model = define_top_model(train_data, num_classes)

	# Define checkpoint callback
	filepath= TOP_MODEL_PATH
	checkpoint = ModelCheckpoint(filepath, monitor='val_acc', 
								verbose=1, save_best_only=True, mode='max')
	# Defining early stopping callback 
	early_stopping = EarlyStopping(monitor='val_loss', min_delta=0, 
								  patience=10, verbose=1, mode='min')
	
	callbacks_list = [checkpoint, early_stopping]

	# train our model
	history = model.fit(train_data, train_labels,
	                    epochs=EPOCHS,
	                    batch_size=BATCH_SIZE,
	                    validation_data=(validation_data, validation_labels),
	                    shuffle=True,
	                    callbacks=callbacks_list,
	                    verbose=1)

	#save the history to *.p file for future review
	pickle.dump(history.history, open( "history_train.p", "wb" ))


if __name__ == '__main__':
	
	train_top_model()
