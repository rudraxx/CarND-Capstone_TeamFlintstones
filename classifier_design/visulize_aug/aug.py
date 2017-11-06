from keras.preprocessing.image import ImageDataGenerator, array_to_img, img_to_array, load_img

import os
filelist = [ f for f in os.listdir('preview') if f.endswith(".png") ]
for f in filelist:
    os.remove(os.path.join('preview', f))

datagen = ImageDataGenerator(rotation_range=15,
							horizontal_flip=True,
							zoom_range=[0.4, 0.99],
							shear_range=0.2,
							width_shift_range=0.1,
        					height_shift_range=0.1)

datagen = ImageDataGenerator()

i = 0
for batch in datagen.flow_from_directory('img_test',
                        target_size=(224, 224),
                        batch_size=20,
                        class_mode='categorical',
                        shuffle=False,
                        save_to_dir='preview',
                        save_prefix='aug',
                        save_format='png'):
    i += 1
    if i > 20:
    	print(i)
        break  # otherwise the generator would loop indefinitely
