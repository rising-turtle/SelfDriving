#!/home/davidz/work/anaconda2/bin/python
## !/usr/bin/python 

import cv2
# from imageio import *
# imageio.plugins.ffmpeg.download()

if __name__=='__main__':
    img = cv2.imread('test.jpg', 0)
    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
