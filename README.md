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
  
FP.3 - FP.4 in this section the task is to associate matches with corresponding bounding boxes and by computing how they shrink or expand over consequent frames. This will allow to compute the camera TTC. In order to address this problem, we first use the function `clusterKptMatchesWithROI` which takes as input the current bounding box, both the previous and current keypoints from the consequent frames together with the corresponding matches. A loop over all the latters is performed and, only if the keypoints are inside the current bounding box, the relative match is pushed into the kptsMatches vector. At this point an outlier filtering is applied in the `computeTTCCamera` function by looping over all the distances between keypoints belonging to the bbox and by computing the distance ratios between all of them. At this point, the median distance ratio is computed to filter out erroneous points. The median distance ration will be used in the TTC final formula which will be now statistically robust against outliers.

FP.5 In this task it is necessary to find some examples where the Lidar TTC estimate is way off the manually computed values. Thanks to the RANSAC filtering, the xmin values we obtain from the lidar estimation are really close to the "real" ones. On the other hand, the structure of the estimation is really sensitive due to the fact that, when minXCurr and minXPrev are almost equal, the denominator tends to zero while the numerator stays positive; this leads to an explosion of the TTC when the movements of the car are small. Moreover, if the data is corrupted by noise, small differences between the real and the measured value lead to consistent estimation errors of the TTC. For instance, between the frames 7 and 8, the movement of the car is really small (approx. 2cm) and, even if the estimated values are almost the true ones, the denominator tends to zero leading to an amplification of noise; in this particular case, the real TTC value is 25.1667, while the estimated one is 34.3404, that is more than 9 seconds of difference. The previous one was the main example, but even between the frames 5 and 6 it is possible to find a similar behavior; here the distance increment is ~4.5cm, but even in this case small errors on the estimation lead to a TTC of 16.2511s against 19.1s (manually computed). Something else to talk of would be the presence of negative times (which phisically do not have a lot of meaning), but they don't represent a real problem; this happens basically when the tracked car is moving away from the ego one leading to a minXCurr greater than the minXPrev. In terms of the TTC formula, this happens because the denominator turns negative (the velocity in the model becomes smaller than zero).  

FP.6
  
