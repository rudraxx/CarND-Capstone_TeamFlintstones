# dimensions of our images.  
IMG_WIDTH, IMG_HEIGHT = 224, 224
 
# name of the file to save our model trained
TOP_MODEL_PATH = 'bottleneck_fc_model.h5'
CAPSTONE_MODEL_PATH = 'capstone_model.h5'

# dirs for data
TRAIN_DATA_DIR = '../data/train'
VALIDATION_DATA_DIR = '../data/validation'

# number of epochs to train top model
EPOCHS = 50
# batch size used by flow_from_directory and predict_generator
BATCH_SIZE = 50