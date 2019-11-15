# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load.
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed.
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson.
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures.

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning.

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Implementation of Mid-Project

1. Data Buffer Optimization
    - First use `push_back` the current frame to the buffer
    - Compare the length of buffer to `dataBufferSize`
    - `erase` the first element in buffer if exceed the `dataBufferSize`

2. Keypoint Detection
    - Set `detectorType` to corresponding detector's name and choose the detect method according to the name string.
    - *Shi-Tomasi* and *Harris* are different from other modern implement so they have their own function. They all need to transfor corner to `KeyPoint`. *Harris* method also need to implement non-maximum suppression by itself.
    - Others such as *FAST* and *ORB* share the function `detKeypointsModern`.

3. Keypoint Removal
    - Using `cv::Rect_::contains` to determine if the keypoint in the `vehicleRect`.

4. Keypoint Descriptors
    - Assign each descriptor to `extractor` by using their own `create()` method according to their name string.
    - Using the common extractor `extractor->compute` to extract the description matrix. _Note: `SIFT` is not free for commercial use_.

5. Descriptor Matching
    - There is a bug when implement FLANN matching, convert binary descriptors to floating point like this `desc.convertTo(desc, CV_32F)`. And according to `selectorType` to use nearest neighbor or k-nearest neighbor.


6. Descriptor Distance Ratio
    - By using `minDescDistRatio` as a threshold to lowering the number of false positives i.e. ambiguous matches. If first match's distance is less than second one multiply by `minDescDistRatio`, that's the keypoint we want.

7. The number of detect keypoints on the preceding vehicle for all 10 images

| detector | keypoints num | detection period | detector | keypoints num | detection period |
| :-----:  | :------:    | :------:       |:---:   | :------:    | :------:       |
| FAST    | 1824        | 0.842811 ms    |ORB     | 500         | 111.738 ms     |
|FAST| 1832| 1.13529 ms|ORB| 500| 6.55984 ms|
|FAST| 1810| 0.662994 ms|ORB| 500| 8.51812 ms|
|FAST| 1817| 0.919915 ms|ORB| 500| 6.75712 ms|
|FAST| 1793| 0.652293 ms|ORB| 500| 6.37889 ms|
|FAST| 1796| 0.816051 ms|ORB| 500| 6.77254 ms|
|FAST| 1788| 0.895189 ms|ORB| 500| 7.14981 ms|
|FAST| 1695| 0.870212 ms|ORB| 500| 7.26431 ms|
|FAST| 1749| 0.809669 ms|ORB| 500| 8.57334 ms|
|FAST| 1770| 0.830207 ms|ORB| 500| 7.46283 ms|
|Shi-Tomasi| 1370| 14.9389 ms|AKAZE| 1351| 83.6214 ms|
|Shi-Tomasi| 1301| 13.427 ms|AKAZE| 1327| 82.6064 ms|
|Shi-Tomasi| 1361| 11.7154 ms|AKAZE| 1311| 82.274 ms|
|Shi-Tomasi| 1358| 15.3105 ms|AKAZE| 1351| 75.2366 ms|
|Shi-Tomasi| 1333| 12.3686 ms|AKAZE| 1360| 65.9895 ms|
|Shi-Tomasi| 1284| 11.895 ms|AKAZE| 1347| 79.2498 ms|
|Shi-Tomasi| 1322| 12.1468 ms|AKAZE| 1363| 77.7107 ms|
|Shi-Tomasi| 1366| 12.4593 ms|AKAZE| 1331| 80.7446 ms|
|Shi-Tomasi| 1389| 11.3764 ms|AKAZE| 1357| 76.9947 ms|
|Shi-Tomasi| 1339| 12.2163 ms|AKAZE| 1331| 77.5664 ms|
|BRISK| 2757| 30.9556 ms|SIFT| 1437| 103.859 ms|
|BRISK| 2777| 34.9054 ms|SIFT| 1371| 74.4862 ms|
|BRISK| 2741| 31.7607 ms|SIFT| 1381| 85.558 ms|
|BRISK| 2735| 30.7006 ms|SIFT| 1336| 72.8097 ms|
|BRISK| 2757| 29.7889 ms|SIFT| 1303| 77.2392 ms|
|BRISK| 2695| 30.7072 ms|SIFT| 1370| 77.3704 ms|
|BRISK| 2715| 37.0952 ms|SIFT| 1396| 70.6995 ms|
|BRISK| 2628| 34.3794 ms|SIFT| 1382| 72.775 ms|
|BRISK| 2639| 33.6813 ms|SIFT| 1462| 77.709 ms|
|BRISK| 2672| 31.1954 ms|SIFT| 1422| 88.799 ms|
|Harris| 115| 18.4843 ms|
|Harris| 98| 16.4098 ms|
|Harris| 113| 10.8225 ms|
|Harris| 121| 12.3457 ms|
|Harris| 160| 13.7682 ms|
|Harris| 383| 21.0132 ms|
|Harris| 85| 12.6305 ms|
|Harris| 210| 11.7116 ms|
|Harris| 171| 14.0952 ms|
|Harris| 281| 15.6142 ms|

8. The number of matched keypoints for all 10 images using all possible combinations

|detector|descriptor|matcher type|selector type|matches num|
| :------: | :------: | :------: |:------: | :------: |
|FAST| BRIEF| MAT_BF| SEL_KNN| 119|
|FAST| BRIEF| MAT_BF| SEL_KNN| 130|
|FAST| BRIEF| MAT_BF| SEL_KNN| 118|
|FAST| BRIEF| MAT_BF| SEL_KNN| 126|
|FAST| BRIEF| MAT_BF| SEL_KNN| 108|
|FAST| BRIEF| MAT_BF| SEL_KNN| 123|
|FAST| BRIEF| MAT_BF| SEL_KNN| 131|
|FAST| BRIEF| MAT_BF| SEL_KNN| 125|
|FAST| BRIEF| MAT_BF| SEL_KNN| 119|
|FAST| BRISK| MAT_BF| SEL_KNN| 97|
|FAST| BRISK| MAT_BF| SEL_KNN| 104|
|FAST| BRISK| MAT_BF| SEL_KNN| 101|
|FAST| BRISK| MAT_BF| SEL_KNN| 98|
|FAST| BRISK| MAT_BF| SEL_KNN| 85|
|FAST| BRISK| MAT_BF| SEL_KNN| 107|
|FAST| BRISK| MAT_BF| SEL_KNN| 107|
|FAST| BRISK| MAT_BF| SEL_KNN| 100|
|FAST| BRISK| MAT_BF| SEL_KNN| 100|
|FAST| ORB| deMAT_BF| SEL_KNN| 118|
|FAST| ORB| deMAT_BF| SEL_KNN| 123|
|FAST| ORB| deMAT_BF| SEL_KNN| 112|
|FAST| ORB| deMAT_BF| SEL_KNN| 126|
|FAST| ORB| deMAT_BF| SEL_KNN| 106|
|FAST| ORB| deMAT_BF| SEL_KNN| 122|
|FAST| ORB| deMAT_BF| SEL_KNN| 122|
|FAST| ORB| deMAT_BF| SEL_KNN| 123|
|FAST| ORB| deMAT_BF| SEL_KNN| 119|
|Shi| BRISK| MAT_BF| SEL_KNN| 95|
|Shi| BRISK| MAT_BF| SEL_KNN| 88|
|Shi| BRISK| MAT_BF| SEL_KNN| 80|
|Shi| BRISK| MAT_BF| SEL_KNN| 90|
|Shi| BRISK| MAT_BF| SEL_KNN| 82|
|Shi| BRISK| MAT_BF| SEL_KNN| 79|
|Shi| BRISK| MAT_BF| SEL_KNN| 85|
|Shi| BRISK| MAT_BF| SEL_KNN| 86|
|Shi| BRISK| MAT_BF| SEL_KNN| 82|
|Shi| BRIEF| MAT_BF| SEL_KNN| 115|
|Shi| BRIEF| MAT_BF| SEL_KNN| 111|
|Shi| BRIEF| MAT_BF| SEL_KNN| 104|
|Shi| BRIEF| MAT_BF| SEL_KNN| 101|
|Shi| BRIEF| MAT_BF| SEL_KNN| 102|
|Shi| BRIEF| MAT_BF| SEL_KNN| 102|
|Shi| BRIEF| MAT_BF| SEL_KNN| 100|
|Shi| BRIEF| MAT_BF| SEL_KNN| 109|
|Shi| BRIEF| MAT_BF| SEL_KNN| 100|
|Shi| ORB| deMAT_BF| SEL_KNN| 106|
|Shi| ORB| deMAT_BF| SEL_KNN| 102|
|Shi| ORB| deMAT_BF| SEL_KNN| 99|
|Shi| ORB| deMAT_BF| SEL_KNN| 102|
|Shi| ORB| deMAT_BF| SEL_KNN| 103|
|Shi| ORB| deMAT_BF| SEL_KNN| 97|
|Shi| ORB| deMAT_BF| SEL_KNN| 98|
|Shi| ORB| deMAT_BF| SEL_KNN| 104|
|Shi| ORB| deMAT_BF| SEL_KNN| 97|
|Shi| FREAK| MAT_BF| SEL_KNN| 86|
|Shi| FREAK| MAT_BF| SEL_KNN| 90|
|Shi| FREAK| MAT_BF| SEL_KNN| 86|
|Shi| FREAK| MAT_BF| SEL_KNN| 88|
|Shi| FREAK| MAT_BF| SEL_KNN| 86|
|Shi| FREAK| MAT_BF| SEL_KNN| 80|
|Shi| FREAK| MAT_BF| SEL_KNN| 81|
|Shi| FREAK| MAT_BF| SEL_KNN| 86|
|Shi| FREAK| MAT_BF| SEL_KNN| 85|
|AKAZE| AKAZE| MAT_BF| SEL_KNN| 138|
|AKAZE| AKAZE| MAT_BF| SEL_KNN| 138|
|AKAZE| AKAZE| MAT_BF| SEL_KNN| 133|
|AKAZE| AKAZE| MAT_BF| SEL_KNN| 127|
|AKAZE| AKAZE| MAT_BF| SEL_KNN| 129|
|AKAZE| AKAZE| MAT_BF| SEL_KNN| 146|
|AKAZE| AKAZE| MAT_BF| SEL_KNN| 147|
|AKAZE| AKAZE| MAT_BF| SEL_KNN| 151|
|AKAZE| AKAZE| MAT_BF| SEL_KNN| 150|
|BRISK| BRISK| MAT_BF| SEL_KNN| 171|
|BRISK| BRISK| MAT_BF| SEL_KNN| 176|
|BRISK| BRISK| MAT_BF| SEL_KNN| 157|
|BRISK| BRISK| MAT_BF| SEL_KNN| 176|
|BRISK| BRISK| MAT_BF| SEL_KNN| 174|
|BRISK| BRISK| MAT_BF| SEL_KNN| 188|
|BRISK| BRISK| MAT_BF| SEL_KNN| 173|
|BRISK| BRISK| MAT_BF| SEL_KNN| 171|
|BRISK| BRISK| MAT_BF| SEL_KNN| 184|
|ORB| ORB| deMAT_BF| SEL_KNN| 67|
|ORB| ORB| deMAT_BF| SEL_KNN| 70|
|ORB| ORB| deMAT_BF| SEL_KNN| 72|
|ORB| ORB| deMAT_BF| SEL_KNN| 84|
|ORB| ORB| deMAT_BF| SEL_KNN| 91|
|ORB| ORB| deMAT_BF| SEL_KNN| 101|
|ORB| ORB| deMAT_BF| SEL_KNN| 92|
|ORB| ORB| deMAT_BF| SEL_KNN| 93|
|ORB| ORB| deMAT_BF| SEL_KNN| 93|
|SIFT| SIFT| dMAT_FLANN| SEL_KNN| 82|
|SIFT| SIFT| dMAT_FLANN| SEL_KNN| 81|
|SIFT| SIFT| dMAT_FLANN| SEL_KNN| 86|
|SIFT| SIFT| dMAT_FLANN| SEL_KNN| 95|
|SIFT| SIFT| dMAT_FLANN| SEL_KNN| 90|
|SIFT| SIFT| dMAT_FLANN| SEL_KNN| 81|
|SIFT| SIFT| dMAT_FLANN| SEL_KNN| 82|
|SIFT| SIFT| dMAT_FLANN| SEL_KNN| 103|
|SIFT| SIFT| dMAT_FLANN| SEL_KNN| 103|
|Harris| SIFT| dMAT_FLANN| SEL_KNN| 14|
|Harris| SIFT| dMAT_FLANN| SEL_KNN| 11|
|Harris| SIFT| dMAT_FLANN| SEL_KNN| 16|
|Harris| SIFT| dMAT_FLANN| SEL_KNN| 19|
|Harris| SIFT| dMAT_FLANN| SEL_KNN| 22|
|Harris| SIFT| dMAT_FLANN| SEL_KNN| 22|
|Harris| SIFT| dMAT_FLANN| SEL_KNN| 13|
|Harris| SIFT| dMAT_FLANN| SEL_KNN| 24|
|Harris| SIFT| dMAT_FLANN| SEL_KNN| 22|
|FAST| SIFT| dMAT_FLANN| SEL_NN| 149|
|FAST| SIFT| dMAT_FLANN| SEL_NN| 152|
|FAST| SIFT| dMAT_FLANN| SEL_NN| 150|
|FAST| SIFT| dMAT_FLANN| SEL_NN| 155|
|FAST| SIFT| dMAT_FLANN| SEL_NN| 149|
|FAST| SIFT| dMAT_FLANN| SEL_NN| 149|
|FAST| SIFT| dMAT_FLANN| SEL_NN| 156|
|FAST| SIFT| dMAT_FLANN| SEL_NN| 150|
|FAST| SIFT| dMAT_FLANN| SEL_NN| 138|
|Harris| BRISK| MAT_BF| SEL_KNN| 12|
|Harris| BRISK| MAT_BF| SEL_KNN| 10|
|Harris| BRISK| MAT_BF| SEL_KNN| 14|
|Harris| BRISK| MAT_BF| SEL_KNN| 15|
|Harris| BRISK| MAT_BF| SEL_KNN| 16|
|Harris| BRISK| MAT_BF| SEL_KNN| 16|
|Harris| BRISK| MAT_BF| SEL_KNN| 15|
|Harris| BRISK| MAT_BF| SEL_KNN| 23|
|Harris| BRISK| MAT_BF| SEL_KNN| 21|
|SIFT| BRISK| MAT_BF| SEL_KNN| 64|
|SIFT| BRISK| MAT_BF| SEL_KNN| 66|
|SIFT| BRISK| MAT_BF| SEL_KNN| 63|
|SIFT| BRISK| MAT_BF| SEL_KNN| 67|
|SIFT| BRISK| MAT_BF| SEL_KNN| 59|
|SIFT| BRISK| MAT_BF| SEL_KNN| 64|
|SIFT| BRISK| MAT_BF| SEL_KNN| 64|
|SIFT| BRISK| MAT_BF| SEL_KNN| 67|
|SIFT| BRISK| MAT_BF| SEL_KNN| 80|

9. Log the time it takes for keypoint detection and descriptor extraction

|descriptor|time cost|descriptor|time cost|descriptor|time cost|
|:----:|:----:|:----:|:----:|:----:|:----:|
|BRIEF| 1.50051 ms|BRISK| 1.77582 ms|ORB| 1.36795 ms|
|BRIEF| 1.01505 ms|BRISK| 1.6262 ms|ORB| 1.75321 ms|
|BRIEF| 1.95739 ms|BRISK| 1.6044 ms|ORB| 0.930716 ms|
|BRIEF| 0.873418 ms|BRISK| 1.35985 ms|ORB| 1.01275 ms|
|BRIEF| 0.75682 ms|BRISK| 1.31194 ms|ORB| 1.51098 ms|
|BRIEF| 1.06062 ms|BRISK| 1.41334 ms|ORB| 1.60942 ms|
|BRIEF| 1.37207 ms|BRISK| 1.43216 ms|ORB| 1.68727 ms|
|BRIEF| 0.838783 ms|BRISK| 1.32005 ms|ORB| 1.28756 ms|
|BRIEF| 0.635171 ms|BRISK| 1.21806 ms|ORB| 1.05243 ms|
|BRIEF| 0.616195 ms|BRISK| 1.4176 ms|ORB| 0.96524 ms|
|FREAK| 40.3625 ms|AKAZE| 69.6459 ms|SIFT| 77.684 ms|
|FREAK| 42.9607 ms|AKAZE| 76.2656 ms|SIFT| 70.9345 ms|
|FREAK| 36.974 ms|AKAZE| 78.6854 ms|SIFT| 68.5022 ms|
|FREAK| 39.0903 ms|AKAZE| 58.4465 ms|SIFT| 70.824 ms|
|FREAK| 36.1779 ms|AKAZE| 65.5983 ms|SIFT| 74.6786 ms|
|FREAK| 37.45 ms|AKAZE| 68.8275 ms|SIFT| 70.8214 ms|
|FREAK| 40.7771 ms|AKAZE| 65.4817 ms|SIFT| 78.0307 ms|
|FREAK| 34.1917 ms|AKAZE| 66.8156 ms|SIFT| 77.119 ms|
|FREAK| 35.1056 ms|AKAZE| 67.4465 ms|SIFT| 68.4407 ms|
|FREAK| 35.2886 ms|AKAZE| 60.9759 ms|SIFT| 63.7122 ms|

10. The TOP3 detector / descriptor combinations for our purpose of detecting keypoints on vehicles.

    The fastest keypoint detector I think is the __FAST__ detector, it only takes less than 1 milliseconds to produces one thousand and several hundreds keypoints thats belong to the upper middle level, so we should take __FAST__ as the detector.
    We should only consider modern descriptor because of the time cost. _SIFT_ and other non-modern descriptor take too long time that not satisfy the vehicle scenes.
    So the last thing is which one should combine with the __FAST__ detector. By comparing the time taken of extraction:
    - _FAST and BRIEF_ is the fastest combination;
    - _FAST and ORB_ is the second one;
    - _FAST and BRISK_ is the third one;
