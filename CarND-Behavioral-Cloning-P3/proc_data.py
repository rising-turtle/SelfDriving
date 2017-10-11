import csv 
import cv2
import numpy as np


def cropImg(img):
    center = img.shape[0]//2, img.shape[1]//2
    img = img[center[0] - 20  : center[0] + 50, 
                center[1] -90 : center[1] + 90]
    return img

lines = []
with open('./data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        lines.append(line)

images = []
measurements = []
for line in lines:
    source_path = line[0]
    if source_path == 'center':
        continue
    img_name = source_path.split('/')[-1]
    curr_path = './data/IMG/' + img_name
    image = cv2.imread(curr_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    img = cropImg(image)
    images.append(img)
    measurement = float(line[-4])
    measurements.append(measurement)

X_train = np.array(images)
y_train = np.array(measurements)

np.save('X_train.npy', X_train)
np.save('y_train.npy', y_train)


