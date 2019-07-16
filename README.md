# SFND 3D Object Tracking



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
4. Run it: `./3D_object_tracking`

# SFND-CameraFinalProject Final Report

FP.1 the matchBoundingBoxes function as been implemented as follows:  
- A voting structure called bins is initialized to zero and its dimensions are the # bboxes in the current frame times  
  the number of bboxes in the previous frame
- Some other support variables are initialized, like a vector to keep track of the "paired" bboxes  
- By looping over all the matches, if the keypoints linked to a match falls into the current and the previous  
  bboxes under test, the corresponding row-column value in the voting structure is incremented (number of occurrences)
- Now the maximum value for every row is taken and together the index associated with it  
- If this combination of bounding boxes has not been paired yet and the number of occurrences is greater than  
  a threshold (in this case set to zero -> at least one occurrence) then the bbox combination is stored in the  
  map.  

FP.2 in order to compute the TTC using lidar, it is necessary to filter out the outliers in order  
  to avoid erroneus measurements for the minimum x value from the cloud. Hence, a ransac fitting  
  a plane has been implemented to match the plane crossing the back of the car and by tuning the  
  tolerance until the obtained values where coherent with manually measured ones. Furthermore, a  
  second check is performed to remove all the points that are outside the ego lane. By finding now  
  the minimum x values in both the clouds it is possible to compute the TTC for the lidar sensor.
  
FP.3 
  
