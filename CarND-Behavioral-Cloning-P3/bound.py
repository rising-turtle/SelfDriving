
import cv2 
import csv 
# from skimage import color, exposure, transform
import matplotlib.pyplot as plt  

lines = []
with open('./data/driving_log.csv') as f:
    reader = csv.reader(f)
    for r in reader:
        lines.append(r)

bd = 100
for line in lines:
    if line[0] == 'center':
        continue
    f = './data/' + line[0]
    print('f = {}'.format(line[0]))
    img = cv2.imread(f)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # plt.imshow(img)
    # plt.show()
    center = img.shape[0]//2, img.shape[1]//2
    img = img[center[0] - 20  : center[0] + 50, 
                center[1] -90 : center[1] + 90]
    # cv2.imshow('test', img)
    # cv2.waitKey(0)
    plt.imshow(img)
    plt.show()
    

