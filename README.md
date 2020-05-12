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



## Task descriptions

To start the variable of string dataPath in MidTermProject_Camera_student.cpp needs to be modified to your local folder path.


#### MP.1 Data Buffer Optimization

Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end.
```
dataBuffer.push_back(frame);
if(dataBuffer.size()>dataBufferSize)
{
    dataBuffer.erase(dataBuffer.begin());
}
```

#### MP.2 Keypoint Detection

Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.
```
detKeypointsModern(keypoints, imgGray, "HARRIS", true);
cv::waitKey(0);
detKeypointsModern(keypoints, imgGray, "FAST", true);
cv::waitKey(0);
detKeypointsModern(keypoints, imgGray, "BRISK", true);
cv::waitKey(0);
detKeypointsModern(keypoints, imgGray, "ORB", true);
cv::waitKey(0);
detKeypointsModern(keypoints, imgGray, "AKAZE", true);
cv::waitKey(0);
detKeypointsModern(keypoints, imgGray, "SIFT", true);
cv::waitKey(0);

```
#### MP.3 Keypoint Removal

Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.
```
for (auto it = keypoints_cut.begin(); it != keypoints_cut.end(); ++it)
{
    //cout << "keypoints x " << it->pt.x << " y is " << it->pt.y <<endl;
    if(it->pt.x>535 && it->pt.y>180 && it->pt.x<535+180 && it->pt.y<180+150)
    {
        keypoints.push_back(*it);
    }
    //std::cout <<"keypoints"<<it->pt.x<< it->pt.y<<endl;
}
```
#### MP.4 Keypoint Descriptors

Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.
```
void descKeypoints()
```
#### MP.5 Descriptor Matching

Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.
```

else if (matcherType.compare("MAT_FLANN") == 0)
{
    if (descSource.type() != CV_32F)
    { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
        descSource.convertTo(descSource, CV_32F);
        descRef.convertTo(descRef, CV_32F);
    }

    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    cout << "FLANN matching";
}


else if (selectorType.compare("SEL_KNN") == 0)
{
}
```


#### MP.6 Descriptor Distance Ratio

Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.
```

double minDescDistRatio = 0.8;
for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
{

    if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
    {
        matches.push_back((*it)[0]);
    }
    
}
```


#### MP.7 Performance Evaluation 1

Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.
check results in results.xlsx
eg.: case, time: 0.146997 match: 11keypoint: 14 det: HARRIS des: BRISK
case, time: 0.146332 match: 9keypoint: 18 det: HARRIS des: BRISK
case, time: 0.137794 match: 10keypoint: 20 det: HARRIS des: BRISK
case, time: 0.138101 match: 11keypoint: 25 det: HARRIS des: BRISK
case, time: 0.154833 match: 16keypoint: 39 det: HARRIS des: BRISK

#### MP.8 Performance Evaluation 2

Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

check results in results.xlsx

eg.: case, time: 0.146997 match: 11keypoint: 14 det: HARRIS des: BRISK
case, time: 0.146332 match: 9keypoint: 18 det: HARRIS des: BRISK
case, time: 0.137794 match: 10keypoint: 20 det: HARRIS des: BRISK
case, time: 0.138101 match: 11keypoint: 25 det: HARRIS des: BRISK
case, time: 0.154833 match: 16keypoint: 39 det: HARRIS des: BRISK

#### MP.9 Performance Evaluation 3

Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.
check results in results.xlsx

eg.: case, time: 0.146997 match: 11keypoint: 14 det: HARRIS des: BRISK
case, time: 0.146332 match: 9keypoint: 18 det: HARRIS des: BRISK
case, time: 0.137794 match: 10keypoint: 20 det: HARRIS des: BRISK
case, time: 0.138101 match: 11keypoint: 25 det: HARRIS des: BRISK
case, time: 0.154833 match: 16keypoint: 39 det: HARRIS des: BRISK

