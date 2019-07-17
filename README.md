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
- A voting structure called bins is initialized to a matrix of zeros and its dimensions are the # bboxes in the current frame   times the number of bboxes in the previous frame
- Some other support variables are initialized, like a vector to keep track of the already "paired" bboxes  
- By looping over all the matches, if the keypoints linked to a match fall into the current and the previous  
  bboxes, the corresponding row-column value in the voting structure is incremented (number of occurrences)
- Now the maximum value for every row is taken and wiith it the associated index
- If this combination of bounding boxes has not been paired yet and the number of occurrences is greater than  
  a threshold (in this case set to zero -> at least one occurrence is required) then the bbox combination is stored in the  
  map.  

FP.2 in order to compute the TTC using lidar, it is necessary to remove the outliers in order  
  to avoid erroneus measurements for the minimum x value in the cloud. Hence, a ransac plane fitting  
  has been implemented to match the back surface of the car. A consequent tuning on tolerance has been  
  performed until the obtained values where coherent or close to the manually measured ones. Furthermore, a  
  second check has been performed to remove all the points that are outside the ego lane. By finding then  
  the minimum x values in both the clouds it has been possible to compute the TTC for the lidar sensor.
  
FP.3 - FP.4 in this section the task is to associate matches with corresponding bounding boxes and by computing how they shrink or expand over consequent frames. If done properly, this should allow the correct computtation of the camera TTC. In order to address this problem, we first use the function `clusterKptMatchesWithROI` which takes as input the current bounding box, both the previous and current keypoints from the consequent frames together with the corresponding matches. A loop over all the latters is performed and, only if the keypoints are inside the current bounding box, the relative match is pushed into the kptsMatches vector. A first attempt to filter outliers has been performed, but there were no justified improvements, so the code has been commented out. In the `computeTTCCamera` function an outlier filtering is applied by looping over all the distances between keypoints belonging to the bbox and by computing the distance ratios between all of them. At this point, the median distance ratio is computed to filter out erroneous matches. The median distance ratio has been used in the TTC final formula to provide a statistically robust estimation.

FP.5 In this task it is necessary to find some examples where the Lidar TTC estimate is way off the manually computed values. Thanks to the RANSAC filtering, the xmin values we obtain from the lidar estimation are really close to the "real" ones. On the other hand, the structure of the estimation formula is really sensitive to noise due to the fact that, when minXCurr and minXPrev are almost equal, the denominator tends to zero while the numerator stays positive; this leads to an explosion of the TTC when the movements of the car are small or close to zero. Moreover, if the data is corrupted by noise, small differences between the real and the measured values could lead to consistent estimation errors of the TTC. For instance, between the frames 7 and 8, the movement of the car is really small (approx. 2cm) and, even if the estimated values are almost equal to the true ones, the denominator tends to zero leading to an amplification of noise (which could be even numerical); in this particular case, the real TTC value is 25.1667, while the estimated one is 34.3404, that is more than 9 seconds of difference. To name another occurrence, even between the frames 5 and 6 it is possible to find a similar behavior; here the distance increment is ~4.5cm and, as before, small errors on the estimation lead to a TTC of 16.2511s against 19.1s. Something else to talk about would be the presence of negative times (which phisically do not have a lot of meaning), but they don't represent a real problem; this happens basically when the tracked car is moving away from the ego one leading to a minXCurr greater than the minXPrev. In terms of the TTC formula, this is allowed and it happens because the denominator turns negative (the velocity in the model becomes smaller than zero).  

FP.6  The previous analisys will be now applied to the camera TTC estimation. To support the following argumentation, a spreadsheet containing all the possible combinations between descriptors and detectors is provided. The document structure is organized as a series of TTC measurements (over frames) associated with a particular pairing; furthermore, the RMSE is computed between the obtained results and the manually computed times. As ground truth, it has been used the lidar TTC formula instead of the camera one mainly because it is not trivial to access the real shrinking or expansion of the car appearance. Hence, the last column of the spreadsheed provides the RMSE with respect to the real lidar TTC estimations. Also, at the bottom of the results it is possible to find a TOP3 table containing the best combinations with respect to RMSE. With a little personal surprise, the AKAZE detector has revealed to be really good for this kind of task; by pairing it with the SIFT descriptor (which is known to be really robust, but computationally expensive), it was possible to maintain a good TTC estimation over frames (or at least stable). Following, we find AKAZE-AKAZE and AKAZE-FREAK and, taking into account the Midterm project output, the SHITOMASI-BRIEF combo has been mentioned mostly because, in terms of computation time and number of matches, it has been proven to be really good. Although the RMSE seems to be low, it results to be big compared to the lidar TTC estimation: as personal conclusion I would say that estimating the compression or expansion of keypoints does not provide as good results as the ones obtained with lidar, which, of course, is natively far more precise. On the other hand, the measurements are somehow plausible and probably, by integrating both the approaches in a sensor fusion algorithm, it would be possible to get an even more precise estimate (compared to the single ones only). By looking now at the spreadsheet, there are many combinations which provide way off estimates: while testing all the possibilities, some algorithms like HARRIS or ORB resulted to be problematic by providing even infinite TTCs; in order to compare "equally" all the combinations, it has been spent some time in tuning each detector/descriptor with different initialization parameters until, by rolling over all the frames, the TTC estimates were different than -infty or NaN. In order to address why some detectors/descriptors provide not plausible TTCs, it would be useful to think about how the camera TTC is computed. At first, a bounding box is provided by the YOLO network, and yet the size might not be stable. Considering now that the bbox dimension contributes to filter matches, this represents a problem in a later computation. Another cause of error is due to the fact that mono camera setups do not have a depth sensing; at the opposite of lidar, here it is not possible to separate keypoint planes, but the best we can do is to remove outliers through a robust euclidean mean computation. Sadly, this usually is not sufficient to take into account all the outliers, especially the ones belonging to different objects but falling in the same bounding box as the tracked car. On top of the previous thoughts, the formula itself suffers from noise: if the ratio between the medDistRatio tends to one, the denominator goes to zero and the TTC explodes; considering now the fact that mono camera estimation is pretty much noisy, small errors on the shrinking/expanding of keypoints computation might lead to the overestimation of time to collision. To conclude this argumentaton, some examples of combinations which provide way off values are ORB-ORB or ORB-FREAK, but this is mostly due to the inability of these pairings to deal with this problem properly. Something else which can be noticed is the absence of negative times in the TTC estimates (on the opposite of lidar). 
  
