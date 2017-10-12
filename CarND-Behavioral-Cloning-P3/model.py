# combine proc_data.py and clone.py
# 
# load ./data and train a CNN for behavior clone
#

import csv 
import cv2
import numpy as np

def cropImg(img):
    center = img.shape[0]//2, img.shape[1]//2
    img = img[center[0] - 20  : center[0] + 50, 
                center[1] -90 : center[1] + 90]
    return img

def handleImg(folder, fname, images, measurement, meas):
    img_name = folder + '/' + fname
    image = cv2.imread(img_name)
    print('read img {}'.format(img_name))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    img = cropImg(image)
    images.append(img)
    measurements.append(measurement)
    # data augmentation
    flip_img = np.fliplr(img)
    images.append(flip_img)
    measurements.append(-measurement)

lines = []
with open('./data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        lines.append(line)

images = []
measurements = []
folder = './data/IMG'
for line in lines:
    source_path = line[0]
    if source_path == 'center':
        continue
    
    # measurement shift: center: 0, left : +0.2, right : -0.2 
    m_shift = [0, 0.2, -0.2]
    for i in range(3): # 0: center 1: left 2: right
        mea = float(line[-4]) + m_shift[i]
        fname = line[i].split('/')[-1]
        handleImg(folder, fname, images, mea, measurements)
        
X_train = np.array(images)
y_train = np.array(measurements)


########### train the CNN model ##############

from keras.models import Sequential 
from keras.layers import Flatten, Dense, Lambda
from keras.layers.core import Dense, Activation, Flatten, Dropout, SpatialDropout2D
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D

model = Sequential()
model.add(Lambda(lambda x: x/255.0 - 0.5, input_shape=(70, 180, 3)))

drop_rt = 0.2

# LeNet
model.add(Conv2D(filters=6, kernel_size=(5,5), padding='same', activation='relu'))  
model.add(MaxPooling2D(pool_size=(2,2)))  
model.add(Dropout(drop_rt))
model.add(Conv2D(filters=16, kernel_size=(5,5), padding='same', activation='relu'))  
model.add(MaxPooling2D(pool_size=(2,2)))
model.add(Dropout(drop_rt))
model.add(Conv2D(filters=28, kernel_size=(5,5), padding='same', activation='relu'))  
model.add(MaxPooling2D(pool_size=(2,2)))
model.add(Dropout(drop_rt))

model.add(Flatten())
model.add(Dense(120))
model.add(Activation('relu'))
model.add(Dropout(drop_rt))
model.add(Dense(84))
model.add(Activation('relu'))
model.add(Dropout(drop_rt))
model.add(Dense(1))

model.compile(loss = 'mse', optimizer = 'adam')
model.fit(X_train, y_train, validation_split = 0.2, shuffle = True, epochs = 3)

model.save('model.h5')
exit()


