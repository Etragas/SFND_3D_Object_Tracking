# SFND 3D Object Tracking

# Student Results

## FP1 Match 3D objects
Done, returns the box id of the best match for each box in prevFrame. This code could be improved to support new objects that don't have a box in prevFrame,

## FP2 Compute Lidar-based TTC
Done, I compute the mean x coordinate of prev lidar points and curr lidar points, and then return the x_diff / v0.

## FP3 Associate Keypoint Correspondences with Bounding Boxes
Done using Rect.contains + filtering all points with average pairwise distance > 300

## FP4 Camera based TTC
Done using the code provided in a previous lesson.

## FP5 Perf 1 Cases Where Lidar is off
### Outlier points throw off estimates
Lidar seems to consistently see *through* the car's bumper. If in a subsequent frame the scanner doesn't detect these through points, our TTC estimates increase due to our constant velocity estimate. 
This is very visible in the frame(18, 19) pair. Where an outlier point causes our TTC to jump from 11.8s -> 8.5s. 
The outlier point is in the top right: 

<img src="https://i.ibb.co/yYkhwSj/image.png"/>

this point then disappears in frame 19 


<img src="https://i.ibb.co/gmKyk20/image.png"/>

### Curvature drops increase perceived velocity
This drop in curvature causes a lorge drop in TTC, from 12s to 9.8s

<img src="https://i.ibb.co/t3b3fth/image.png" width="1440" height="720" />

## FP6 Perf 2
## MP7 Performance Evaluation 1
This sheet records the prediction of each combo at each frame. The best runs are highlighted. 
https://docs.google.com/spreadsheets/d/1UjGdDDPwwlU-jJFoK2x6S4atTt9iF40zgKUWre7tebg/edit?usp=sharing
I then cleaned up the data to remove some outlier predictions, and graphed the best performers:
https://docs.google.com/spreadsheets/d/1UjGdDDPwwlU-jJFoK2x6S4atTt9iF40zgKUWre7tebg/edit#gid=30454966

<img src="https://docs.google.com/spreadsheets/d/e/2PACX-1vTb-sW4quK13SZPhAsvSZPGDzAQ8-CLDRsMojmsSMTpxKUk05-IVCPB2aKlu5fRDVG7Le8iyJE9MxPQ/pubchart?oid=951373265&format=image" />

We can see that FAST+SIFT performed well when compared to LIDAR. This is aligned with out results from Project 2, where Fast + Sift was our #1 pick.

# End of Student Results
## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
