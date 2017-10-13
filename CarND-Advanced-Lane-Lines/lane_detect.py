
import cv2 
import numpy as np
import matplotlib.pyplot as plt  

def abs_sobel_thresh(img, orient='x', kernel_size= 3, thresh=(0,255)):
    # Convert to grayscale 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Sobel gradient
    sobel = []
    if orient == 'x':
        sobel = cv2.Sobel(gray, cv2.CV_64F, 1, 0, kernel_size)
    else:
        sobel = cv2.Sobel(gray, cv2.CV_64F, 0, 1, kernel_size)
    # Sobel absolute 
    sobel_abs = np.absolute(sobel)
    # binary mask using the threshold 
    bin_output = np.zeros_like(sobel_abs)
    bin_output[(sobel_abs >= thresh[0]) & (sobel_abs <= thresh[1])] = 1
    
    return bin_output 

def mag_thresh(img, kernel_size = 3, thresh=(0, 255)):
    # Convert to grayscale 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Sobel gradient
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, kernel_size)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, kernel_size)
    # Sobel magnitude 
    sobel_mag = np.sqrt(np.square(sobelx) + np.square(sobely))
    # scaled 
    sobel_scale = np.uint8(255 * sobel_mag/np.max(sobel_mag))

    # binary mask using the threshold 
    bin_output = np.zeros_like(sobel_scale)
    bin_output[(sobel_scale >= thresh[0]) & (sobel_scale <= thresh[1])] = 1
    
    return bin_output 

def dir_thresh(img, kernel_size = 3, thresh=(0, 1.)):
    # Convert to grayscale 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Sobel gradient
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, kernel_size)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, kernel_size)
    # Sobel orientation 
    sobel_ori = np.arctan2(np.absolute(sobely), np.absolute(sobelx))

    # binary mask using the threshold 
    bin_output = np.zeros_like(sobel_ori)
    bin_output[(sobel_ori >= thresh[0]) & (sobel_ori <= thresh[1])] = 1
    
    return bin_output 

# Choose a Sobel kernel size 
ksize = 3 # Choose a larger odd number to smooth gradient measurements 

image = cv2.imread('signs_vehicles_xygrad.jpg')

plt.imshow(image)
plt.show()

# Apply each of the thresholding functions 
# gradx = abs_sobel_thresh(image, orient='x', kernel_size = ksize, thresh=(0, 255))
# grady = abs_sobel_thresh(image, orient='y', kernel_size = ksize, thresh=(0, 255))
mag_bin = mag_thresh(image, ksize, thresh = (70, 100))
# dir_bin = dir_thresh(image, ksize, thresh = (0, np.pi/2))

plt.imshow(mag_bin)
plt.show()

# combined = np.zeros_like(dir_bin)
# combined[((gradx == 1) & (grady == 1)) | ((mag_bin == 1) & (dir_bin == 1))]= 1









