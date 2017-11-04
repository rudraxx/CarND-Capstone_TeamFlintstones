"""
This script makes the prediction step


Author: @jjaviergalvez

References: 
[1] www.codesofinterest.com/2017/08/bottleneck-features-multi-class-classification-keras.html
[2] https://blog.keras.io/building-powerful-image-classification-models-using-very-little-data.html


"""

#import os.path
#os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

from keras.applications.mobilenet import MobileNet
from keras.preprocessing import image
from keras.applications.mobilenet import preprocess_input, decode_predictions
from keras.models import load_model
from keras import optimizers
from keras.preprocessing.image import ImageDataGenerator
from keras.models import Model
from keras.callbacks import ModelCheckpoint, EarlyStopping
from keras.models import Sequential

#import numpy as np


from consts import *

def get_generators():
	# prepare data augmentation configuration we only using
	# horizontal flip as data augmentation
	train_datagen = ImageDataGenerator(
		preprocessing_function=preprocess_input,
		rotation_range=15,
		horizontal_flip=True,
		zoom_range=[0.4, 0.99],
		shear_range=0.2,
		width_shift_range=0.1,
		height_shift_range=0.1)

	train_generator = train_datagen.flow_from_directory(
		TRAIN_DATA_DIR,
		target_size=(IMG_HEIGHT, IMG_WIDTH),
		batch_size=BATCH_SIZE,
		class_mode='categorical')

	validation_datagen = ImageDataGenerator(
		preprocessing_function=preprocess_input)

	validation_generator = validation_datagen.flow_from_directory(
		VALIDATION_DATA_DIR,
		target_size=(IMG_HEIGHT, IMG_WIDTH),
		batch_size=BATCH_SIZE,
		class_mode='categorical')

	nb_train_samples = len(train_generator.filenames)
	nb_validation_samples = len(validation_generator.filenames)

	return train_generator, validation_generator, nb_train_samples, nb_validation_samples

def define_model():
	# load the MobileNet model
	base_model = MobileNet(input_shape=(IMG_WIDTH, IMG_HEIGHT, 3),
	                  include_top=False, 
	                  weights='imagenet')
	
	# load our classfier model
	top_model = load_model(TOP_MODEL_PATH)

	# This idea comes from https://github.com/fchollet/keras/issues/4040
	new_model = Sequential()
	for layer in base_model.layers:
		new_model.add(layer)

	for layer in top_model.layers:
		new_model.add(layer)


	# set the first 70 layers to non-trainable (weights will not be updated)
	# In the paper https://arxiv.org/pdf/1704.04861.pdf this 70 layers are
	# the first 14 rows of the table 1.
	for layer in new_model.layers[:70]:
	    layer.trainable = False
	#for layer in new_model.layers:
	#	print(layer.name, layer.trainable)
	
	#new_model.summary()

	# compile the model with a SGD/momentum optimizer
	# and a very slow learning rate.
	new_model.compile(loss='binary_crossentropy',
	          optimizer=optimizers.SGD(lr=1e-4, momentum=0.9),
	          metrics=['accuracy'])

	return new_model

def fine_tuning_train():
	train_generator, validation_generator, nb_train_samples, nb_validation_samples = get_generators()

	model = define_model()

	
	# Define checkpoint callback
	filepath= CAPSTONE_MODEL_PATH
	checkpoint = ModelCheckpoint(filepath, monitor='val_acc', 
								verbose=1, save_best_only=True, mode='max')
	# Defining early stopping callback 
	early_stopping = EarlyStopping(monitor='val_loss', min_delta=0, 
								  patience=10, verbose=1, mode='min')
	
	callbacks_list = [checkpoint, early_stopping]


	history = model.fit_generator(
	    train_generator,
	    steps_per_epoch=nb_train_samples // BATCH_SIZE,
	    epochs=EPOCHS,
	    validation_data=validation_generator,
	    validation_steps=nb_validation_samples // BATCH_SIZE,
	    callbacks=callbacks_list)

	#save the history to *.p file for future review
	pickle.dump(history.history, open( "history_train_finetune.p", "wb" ))
	

fine_tuning_train()

