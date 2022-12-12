# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
1. cmake >= 2.8
 * All OSes: [click here for installation instructions](https://cmake.org/install/)

2. make >= 4.1 (Linux, Mac), 3.81 (Windows)
 * Linux: make is installed by default on most Linux distros
 * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
 * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

3. OpenCV >= 4.1
 * All OSes: refer to the [official instructions](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html)
 * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors. If using [homebrew](https://brew.sh/): `$> brew install --build-from-source opencv` will install required dependencies and compile opencv with the `opencv_contrib` module by default (no need to set `-DOPENCV_ENABLE_NONFREE=ON` manually). 
 * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)

4. gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using either [MinGW-w64](http://mingw-w64.org/doku.php/start) or [Microsoft's VCPKG, a C++ package manager](https://docs.microsoft.com/en-us/cpp/build/install-vcpkg?view=msvc-160&tabs=windows). VCPKG maintains its own binary distributions of OpenCV and many other packages. To see what packages are available, type `vcpkg search` at the command prompt. For example, once you've _VCPKG_ installed, you can install _OpenCV 4.1_ with the command:
```bash
c:\vcpkg> vcpkg install opencv4[nonfree,contrib]:x64-windows
```
Then, add *C:\vcpkg\installed\x64-windows\bin* and *C:\vcpkg\installed\x64-windows\debug\bin* to your user's _PATH_ variable. Also, set the _CMake Toolchain File_ to *c:\vcpkg\scripts\buildsystems\vcpkg.cmake*.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

### WriteUp
## MP.1 Data Buffer Optimization
_Lines 102 - 114 in MidTermProject_Camera_Student.cpp_
A Cyclic Ring buffer was implemented with a length of 2 to avoid unnecessary saved images in the buffer.

## MP.2 Keypoint Detection
_Lines 153 - 280 in matching2D_Student.cpp_
Implemented detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT with the default values and made them selectable by setting a string accordingly.

## MP.3 Keypoint Removal
_Lines 155 - 168 in MidTermProject_Camera_Student.cpp_
Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.

## MP.4 Keypoint Descriptors
_Lines 63 - 110 in matching2D_Student.cpp_
Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.

## MP.5 Descriptor Matching
_Lines 7 - 60 in matching2D_Student.cpp_
Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.

## MP.6 Descriptor Distance Ratio
_Lines 7 - 60 in matching2D_Student.cpp_
Added KNN match selection and perform descriptor distance ratio filtering with t=0.8

## MP.7 Performance Evaluation 1
The performance results are presented in Results.xlsx
<img src="images/Table1.PNG" width="820" height="248" />

## MP.8 Performance Evaluation 2
The performance results are presented in Results.xlsx
<img src="images/Table2.PNG" width="820" height="248" />

## MP.9 Performance Evaluation 3
The performance results are presented in Results.xlsx
<img src="images/Table3.PNG" width="820" height="248" />

#### Detector / Descriptor Recommendations

The combination recommendation is based on a score function with the highest score to the combination with the most matching keypoints and least runtime. 
                                        
                                        Score = 10 * (Mean_Matching_Keypoints/Mean_Time) 

* Rank 1 : FAST - BRIEF. 
* Rank 2 : FAST - ORB. 
* Rank 3 : ORB  - BRIEF. 
