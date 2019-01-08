# **Finding Lane Lines on the Road** 

## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file. But feel free to use some other method and submit a pdf if you prefer.

---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./examples/grayscale.jpg "Grayscale"

---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 7 steps. 
1. convert the images to grayscale
2. call guassian blur as kernal size 3 for removing noise on the grayscale image
3. use canny edge detection on blurred image
4. apply the region mask on the canny edges so i can focus on the lane lines
5. calculate hough line from masked region
6. draw hough lines image
7. overlay hough line image on original image

In order to draw a single line on the left and right lanes, I modified the draw_lines() function by ...

1. calculate slope for each line
2. filter out stepp vertical lines by checking slope, also use slope to detemine the side of the line. i.c. left line has a negative slope while right line has a positive slope
3. mataine a slope array and center array for left side and right side
4. average the slope and center for each side
5. extrapolate top point and bottom point for each side by using the average slope and center and i know i am going to reach the bottom of the image for each side.
6. draw a singe line for left side and a single line for right side

### 2. Identify potential shortcomings with your current pipeline


The solution uses fixed region mask for lane, what if the camera changed the orientation or position?

and what if the lane mark is vague?


### 3. Suggest possible improvements to your pipeline

Deep learning could be a more solid solution as far as i can tell.
