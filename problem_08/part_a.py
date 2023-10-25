import glob
import os
import tensorflow as tf
from tensorflow.keras.applications.vgg16 import VGG16
from tensorflow.keras.applications.vgg16 import preprocess_input, decode_predictions
from tensorflow.keras.preprocessing import image
import numpy as np

# Load the pre-trained VGG16 model
model = VGG16(weights='imagenet')

# Directory where your images are stored
image_folder = '/home/yotta/class/inteligent_robots/CS5510-R2D2/problem_08/grocerystore'

# Use glob to find all JPEG files in the folder
image_files = glob.glob(os.path.join(image_folder, '*.jpg'))


# Loop through the images in the folder
for img_path in image_files:
    img = image.load_img(img_path, target_size=(224, 224))
    img = image.img_to_array(img)
    img = preprocess_input(img)
    img = np.expand_dims(img, axis=0)

    # Make predictions
    predictions = model.predict(img)
    decoded_predictions = decode_predictions(predictions, top=3)[0]

    print(f'Image: {os.path.basename(img_path)}')
    for _, label, score in decoded_predictions:
        print(f'{label}: {score:.2f}')
    print('-' * 20)