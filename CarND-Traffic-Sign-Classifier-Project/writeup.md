#**Traffic Sign Recognition** 

##Writeup Template

###You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./examples/visualization.jpg "Visualization"
[image2]: ./examples/grayscale.jpg "Grayscaling"
[image3]: ./examples/random_noise.jpg "Random Noise"
[image4]: ./test_imgs/00181.jpg "Traffic Sign 1"
[image5]: ./test_imgs/00198.jpg "Traffic Sign 2"
[image6]: ./test_imgs/00673.jpg "Traffic Sign 3"
[image7]: ./test_imgs/01961.jpg "Traffic Sign 4"
[image8]: ./test_imgs/00034.ppm "Traffic Sign 5"

## Rubric Points
###Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/481/view) individually and describe how I addressed each point in my implementation.  

---
###Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. You can use this template as a guide for writing the report. The submission includes the project code.

You're reading it! and here is a link to my [project code](https://github.com/udacity/CarND-Traffic-Sign-Classifier-Project/blob/master/Traffic_Sign_Classifier.ipynb)

###Data Set Summary & Exploration

####1. Provide a basic summary of the data set. In the code, the analysis should be done using python, numpy and/or pandas methods rather than hardcoding results manually.

I used the pandas library to calculate summary statistics of the traffic
signs data set:

* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is (32, 32, 3)
* The number of unique classes/labels in the data set is 43

####2. Include an exploratory visualization of the dataset.

Here is an exploratory visualization of the data set. It is a bar chart showing how the data classes distribute in the dataset.

![alt text][image1]

###Design and Test a Model Architecture

####1. Describe how you preprocessed the image data. What techniques were chosen and why did you choose these techniques? Consider including images showing the output of each preprocessing technique. Pre-processing refers to techniques such as converting to grayscale, normalization, etc. (OPTIONAL: As described in the "Stand Out Suggestions" part of the rubric, if you generated additional data for training, describe why you decided to generate additional data, how you generated the data, and provide example images of the additional data. Then describe the characteristics of the augmented training set like number of images in the set, number of images for each class, etc.)

As a first step, I shuffled the training and validation datasets by defining a shuffle_data() function because each time, a batch of the dataset is sent to train the CNN model. In order to avoid using only few classes in a batch for training, which will result into overfitted CNN model for these few classes, shuffle operation is needed to make each batch contain large number of classes.

Then, I run histogram equalization that will normalize the image's value between 0~1 and increase the contrastness of the image. After that, I resize the image to 32*32. 

####2. Describe what your final model architecture looks like including model type, layers, layer sizes, connectivity, etc.) Consider including a diagram and/or table describing the final model.

My final model consisted of the following layers:

| Layer         		|     Description	        					| 
|:---------------------:|:---------------------------------------------:| 
| Input         	| 3x32x32 RGB image   							| 
| Convolution 3x3     	| 1x1 stride, SAME padding, outputs 32x32x32 	|
| RELU			|												|
| Max pooling	      	| 2x2 stride,  outputs 32x16x16 				|
| Convolution 3x3     	| 1x1 stride, SAME padding, outputs 64x16x16 	|
| RELU			|												|
| Max pooling	      	| 2x2 stride,  outputs 64x8x8 				|
| Convolution 3x3     	| 1x1 stride, SAME padding, outputs 128x16x16 	|
| RELU			|												|
| Max pooling	      	| 2x2 stride,  outputs 128x4x4 				|
| Flatten fc0	| -1*2048     									|
| Fully connected fc1	| -1*256 dropout   0.5 	relu								|
| Fully connected fc2	| -1*43     									|
| Softmax		| etc.        									|



####3. Describe how you trained your model. The discussion can include the type of optimizer, the batch size, number of epochs and any hyperparameters such as learning rate.

The optimizer is choosen as Adam, since generally it performs best as discussed in this blog http://ruder.io/optimizing-gradient-descent/index.html#adam.

The batch size is 32, which I have not tuned this parameter to compare. 
Epochs is set as 7, but I see that in the 2nd iteration, the validation accuracy already converge to about ~94.4. So, epochs should be 2 will work in this case. 

####4. Describe the approach taken for finding a solution and getting the validation set accuracy to be at least 0.93. Include in the discussion the results on the training, validation and test sets and where in the code these were calculated. Your approach may have been an iterative process, in which case, outline the steps you took to get to the final solution and why you chose those steps. Perhaps your solution involved an already well known implementation or architecture. In this case, discuss why you think the architecture is suitable for the current problem.

My final model results were:
* training set accuracy of 
* validation set accuracy of 96.28%  
* test set accuracy of 95.76% 

If an iterative approach was chosen:
* What was the first architecture that was tried and why was it chosen? 
* What were some problems with the initial architecture?
* How was the architecture adjusted and why was it adjusted? Typical adjustments could include choosing a different model architecture, adding or taking away layers (pooling, dropout, convolution, etc), using an activation function or changing the activation function. One common justification for adjusting an architecture would be due to overfitting or underfitting. A high accuracy on the training set but low accuracy on the validation set indicates over fitting; a low accuracy on both sets indicates under fitting.

** As constructed in https://chsasank.github.io/keras-tutorial.html, but I observe that the validation accuracy soon get about ~96, and the rest iteration may not contribute new improvement. This phenonmenon may indicate a probable overfitting case. I simplify the structure by reducing the dense layers and cnn layers, which still work pretty good at ~94.4.

* Which parameters were tuned? How were they adjusted and why?
** epochs from 7 to 2, since it converges in 2nd round. 
batch_size, a larger value 128 may not increase accuracy. 
using keras with 'adam' optimizer, using default lr = 0.0001 will work 
learning_rate from 0.0001 to 0.0003 since the accuracy achieves almost steady at epochs = 15 with 0.0001

* What are some of the important design choices and why were they chosen? For example, why might a convolution layer work well with this problem? How might a dropout layer help with creating a successful model?


If a well known architecture was chosen:
* What architecture was chosen?
* Why did you believe it would be relevant to the traffic sign application?
* How does the final model's accuracy on the training, validation and test set provide evidence that the model is working well? 

It works well with about 96.28% validation accuracy and 95.76% test accuracy. 

###Test a Model on New Images

####1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are one German traffic signs that I found on the web:
![alt text][image7] 

The last two images might be difficult to classify because 30% portion of this image has been blackened. 
![alt text][image5] ![alt text][image6] 

The sixth image is difficult too due to large blurring effect.
![alt text][image4] 

####2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set (OPTIONAL: Discuss the results in more detail as described in the "Stand Out Suggestions" part of the rubric).

Here are the results of the prediction:

| Image			        |     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| Stop Sign      		| Stop sign   									|
| Dangerous curve to the right      		| Dangerous curve to the right								| 
| Speed limit (30km/h)  | Speed limit (30km/h)  										|
| Wild animals crossing					| Wild animals crossing											|
| Speed limit (50km/h)      		| Speed limit (50km/h)					 				|
|Speed limit (70 km/h)			| Speed limit (70 km/h)      							|
| Ahead alone		| Ahead alone     							|
| General Caution		|General Caution     							|


The model was able to correctly guess 5 of the 5 traffic signs, which gives an accuracy of 100%. 

####3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

The code for making predictions on my final model is located in the 11th cell of the Ipython notebook.

For the first image, the model is relatively sure that this is a stop sign (probability of 0.6), and the image does contain a stop sign. The top five soft max probabilities were

| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| 6.08%     		| Stop sign   									| 
| 6.06%    		| Dangerous curve to the right								| 
| 6.08% | Speed limit (30km/h)  										|
| 6.08%				| Wild animals crossing											|
| 6.08%    		| Speed limit (50km/h)					 				|
| 6.08%		| Speed limit (70 km/h)      							|
| 6.08%		| Ahead alone     							|
| 3.53%		|General Caution     							|

For the other images, plesase see the result in the .ipynb file. 

### (Optional) Visualizing the Neural Network (See Step 4 of the Ipython notebook for more details)
####1. Discuss the visual output of your trained network's feature maps. What characteristics did the neural network use to make classifications?

