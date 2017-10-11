#!/usr/bin/python
# 
# dirve behavior clone
#

import numpy as np 

# load numpy array data for training 

X_train = np.load('X_train.npy')
y_train = np.load('y_train.npy')

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

model.add(Flatten())
model.add(Dense(120))
model.add(Activation('relu'))
model.add(Dropout(drop_rt))
model.add(Dense(84))
model.add(Activation('relu'))
model.add(Dropout(drop_rt))
model.add(Dense(1))

model.compile(loss = 'mse', optimizer = 'adam')
model.fit(X_train, y_train, validation_split = 0.2, shuffle = True, epochs = 2)

model.save('model.h5')
exit()


