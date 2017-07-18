
**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./examples/grayscale.jpg "Grayscale"

---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps. 
* First, I converted the images to grayscale;
* Second, I applied gaussion filter to reduce noise in the gray image; 
* Third, I extracted edges using canny function; 
* Fourth, I defined and extracted a triangle area which contains the lane lines;
* Fifth, I detected lane lines with specific parameters. 

In order to draw a single line on the left and right lanes, I modified the draw_lines() function by
* First, I separated the line segments into two line sets: left lines and right lines; 
* Then, I compute a single left and right lane line given these two line sets; 

### 2. Identify potential shortcomings with your current pipeline

One potential shortcoming would be what would happen when an outlier line segment is included in the line sets, which can introduce large error for the lane line. 

Another shortcoming could be the case when the triangle area could not contain the lane lines, which may happen when the vehicle turns a large degree. 


### 3. Suggest possible improvements to your pipeline

A possible improvement would be to statistically estimate the true left and right lane line given the line sets by removing outlier lines. 

Another potential improvement could be to define a more accurate polygon for which can better bounds the lane lines than a triangle. 
