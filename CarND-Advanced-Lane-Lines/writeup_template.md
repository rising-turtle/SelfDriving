## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/undistorted.png "Undistorted"
[image2]: ./output_images/threshod.png "Thresh Comparison"
[image3]: ./output_images/threshod2.png "Binary Example"
[image4]: ./output_images/birdeye-view.png "Warp Example"
[image5]: ./output_images/lane_linae.png "Fit Visual"
[image6]: ./output_images/lane_detection.png "Output"
[image7]: ./output_images/lane_detection2.png "Output2"
[video1]: ./output_images/project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You are reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the 1st code cell of the IPython notebook located in "advanced_lane_detection.ipynb".  

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image "./camera_cal/calibration1.jpg" using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

# To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one: (TODO)


#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

[image2]: ./output_images/threshod.png "Thresh Comparison"
[image3]: ./output_images/threshod2.png "Binary Example"
[image_color]: ./output_images/color_threshod.png "Color Thresh"

I separately tested the result using gradient x, gradient magnitude, gradient direction and color threshold in 'S' space: 
![alt text][image2]
This figure indicates that using color threshold can retain the major part of the lane line and the missed part can be retrived by using the gradient threshod. The result from gradient x and gradient magitude seems similar. Therefore, I used a combination of color, gradient magnitude and gradient direction to generate a binary image. This step is implemtned in the 2nd code cell of the notebook. Here is an example of my output for this step: 
![alt text][image3]

As advised by reviewers, using color threshold, specifically extracting yellow & white pixels, can robustly detect lane line even in shadows. One example is shown below: 
![alt_text][image_color]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform is written in the 4th cell of the notebook, which  includes a function called `warper()`, which first choose four point pairs named as [src, dst] to compute the transformation matrix and second use this matrix to transform a image in the birdeye view. I chose the hardcode the source and destination points in the following manner:

```python
top_left_src = [565,470]
top_right_src = [720,470]
bottom_right_src = [1130,720]
bottom_left_src = [200,720]

src = np.float32([top_left_src, top_right_src, bottom_right_src, bottom_left_src])

top_left_dst = [320,0]
top_right_dst = [980,0]
bottom_right_dst = [980,720]
bottom_left_dst = [320,720]

dst = np.float32([top_left_dst, top_right_dst, bottom_right_dst, bottom_left_dst])
```

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 565, 470      | 320, 0        | 
| 720, 470      | 980, 0        |
| 1130, 720     | 980, 720      |
| 200, 720      | 320, 720      |

[image4]: ./output_images/birdeye-view.png "Warp Example"
[image5]: ./output_images/lane_linae.png "Fit Visual"
[image6]: ./output_images/lane_detection.png "Output"
[image7]: ./output_images/lane_detection2.png "Output2"

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

Then I did some other stuff and fit my lane lines with a 2nd order polynomial (in the 5th,6th and 7th code cell) kinda like this:

![alt text][image5]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did this in the the 8th and 9th code cell in my notebook. 

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

In the 10th cell, I demonstarte the output of the implemented pipeline using an image "./test_images/test1.jpg", showing as: 
![alt text][image6]

Then I wraped up all the above modules in the class Line in the 11th code cell, which tightly implemented the pipelines in the function lane_line_detect(). Next, in the 12th code cell, I tested this function with the input image "./test_images/test1.jpg", and got the result showing as:
![alt text][image7]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here is a [link to my video result](./output_images/solidYellowLeft.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

The results show that using color threshold (yellow & white) superiors to using gradient threshold (magnitude & orientation), which weaks the significance of gradient. However, as discussed in the course, lane detection using color is subject to illumination, which may varies in different weather conditions, such as snow, rain or foggy. In these cases, the color based method may fail. Further stategies are needed for improvement. 
