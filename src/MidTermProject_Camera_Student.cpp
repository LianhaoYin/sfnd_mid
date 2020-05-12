/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    
    /* INIT VARIABLES AND DATA STRUCTURES */
    
    // data location
    string dataPath = "/Users/lianhao/OneDrive/CodePath/DL/submission/SFND_mid/";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)
    
    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = true;            // visualize results
    
    
    string detectorType = "FAST"; //HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    string descriptorType = "BRISK"; // BRISK, ORB, FREAK, AKAZE, SIFT
    string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN
    
    std::vector<string> detectorType_all;
    detectorType_all.push_back("HARRIS");
    detectorType_all.push_back("FAST");
    detectorType_all.push_back("BRISK");
    detectorType_all.push_back("ORB");
    detectorType_all.push_back("SIFT");
    detectorType_all.push_back("AKAZE");
    
    std::vector<string> descriptorType_all;
    descriptorType_all.push_back("BRISK");
    descriptorType_all.push_back("ORB");
    descriptorType_all.push_back("FREAK");
    descriptorType_all.push_back("SIFT");
    
    std::vector<double> time_all;
    std::vector<double> matchnr_all;
    std::vector<double> keypoint_all;

    std::vector<string> des_result_all;
    std::vector<string> det_result_all;
    
    for(int i_det = 0; i_det <6;i_det++)
    {
        
        for(int i_des = 0;i_des<4;i_des++)
        {

            
            if(i_det ==5)
            {
                detectorType = "AKAZE";
                descriptorType = "AKAZE";
            }
            else
            {
                detectorType = detectorType_all[i_det];
                descriptorType = descriptorType_all[i_des];
            }
            //string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
            if(detectorType=="HARRIS"&&descriptorType=="FREAK")
            {
                continue;
            }
            if(detectorType=="HARRIS"&&descriptorType=="SIFT")
            {
                continue;
            }
            if(detectorType=="FAST"&&descriptorType=="FREAK")
            {
                continue;
            }
            if(detectorType=="BRISK"&&descriptorType=="SIFT")
            {
                continue;
            }
            if(detectorType=="SIFT"&&descriptorType=="ORB")
            {
                continue;
            }
            
            try {
                cout << "...................................begin........................."<<endl;
                cout << detectorType <<"..........."<<descriptorType<<"..........." << endl;

                //std::vector<double> time_log;
                std::vector<double> sample_log;
                
                
                /* MAIN LOOP OVER ALL IMAGES */
                //double time_log = (double)cv::getTickCount(); // start time counting
                
                
                
                for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
                {
                    double time_log = (double)cv::getTickCount(); // start time counting

                    /* LOAD IMAGE INTO BUFFER */
                    
                    // assemble filenames for current index
                    ostringstream imgNumber;
                    imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                    string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;
                    
                    // load image from file and convert to grayscale
                    cv::Mat img, imgGray;
                    img = cv::imread(imgFullFilename);
                    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
                    
                    //// STUDENT ASSIGNMENT
                    //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
                    
                    // push image into data frame buffer
                    DataFrame frame;
                    frame.cameraImg = imgGray;
                    dataBuffer.push_back(frame);
                    if(dataBuffer.size()>dataBufferSize)
                    {
                        dataBuffer.erase(dataBuffer.begin());
                    }
                    
                    std:: cout <<" buffer size is " <<dataBuffer.size()<< " " << endl;
                    
                    //// EOF STUDENT ASSIGNMENT
                    cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;
                    
                    /* DETECT IMAGE KEYPOINTS */
                    
                    // extract 2D keypoints from current image
                    vector<cv::KeyPoint> keypoints; // create empty feature list for current image
                    
                    //// STUDENT ASSIGNxMENT
                    //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                    //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
                    
                    bool b_plotdifferenmethod = false;
                    if(b_plotdifferenmethod)
                    {
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
                    }
                    
                    //// EOF STUDENT ASSIGNMENT
                    
                    //// STUDENT ASSIGNMENT
                    //// TASK MP.3 -> only keep keypoints on the preceding vehicle
                    
                    // only keep keypoints on the preceding vehicle
                    bool bFocusOnVehicle = true;
                    cv::Rect vehicleRect(535, 180, 180, 150);
                    if (bFocusOnVehicle)
                    {
                        //cv::Mat imgGray_temp = imgGray(vehicleRect);
                        vector<cv::KeyPoint> keypoints_cut; // create empty feature list for current image
                        detKeypointsModern(keypoints_cut, imgGray, detectorType, false);
                        
                        std::cout << "modern keypoints_cut number are " << keypoints_cut.size() << endl;
                        
                        for (auto it = keypoints_cut.begin(); it != keypoints_cut.end(); ++it)
                        {
                            //cout << "keypoints x " << it->pt.x << " y is " << it->pt.y <<endl;
                            if(it->pt.x>535 && it->pt.y>180 && it->pt.x<535+180 && it->pt.y<180+150)
                            {
                                keypoints.push_back(*it);
                            }
                            //std::cout <<"keypoints"<<it->pt.x<< it->pt.y<<endl;
                        }
                        std::cout << "modern keypoints number are " << keypoints.size() << endl;
                        std::cout <<"finished detKeypointsModern"<<endl;
                        
                        cv::Mat visImage = imgGray.clone();
                        cv::drawKeypoints(imgGray, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                        string windowName = "windows show the ";
                        cv::namedWindow(windowName, 7);
                        imshow(windowName, visImage);
                        //cv::waitKey(0);
                        
                        //std::cout <<"finished show the second assignment"<<endl;
                        
                    }
                    
                    sample_log.push_back(keypoints.size()); // save keypoint size
                    //// EOF STUDENT ASSIGNMENT
                    
                    // optional : limit number of keypoints (helpful for debugging and learning)
                    bool bLimitKpts = false;
                    if (bLimitKpts)
                    {
                        
                        int maxKeypoints = 10;
                        
                        if (detectorType.compare("SHITOMASI") == 0)
                        { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                            keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                        }
                        cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                        cout << " NOTE: Keypoints have been limited!" << endl;
                    }
                    
                    // push keypoints and descriptor for current frame to end of data buffer
                    (dataBuffer.end() - 1)->keypoints = keypoints;
                    cout << "#2 : DETECT KEYPOINTS done" << endl;
                    
                    /* EXTRACT KEYPOINT DESCRIPTORS */
                    
                    //// STUDENT ASSIGNMENT
                    //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                    //// -> BRIEF, ORB, FREAK, AKAZE, SIFT
                    
                    cv::Mat descriptors;
                    //string descriptorType = "BRISK"; // BRISK, ORB, FREAK, AKAZE, SIFT
                    cout << "input descriptortype is " << descriptorType << "check if selected"<<endl;
                    descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
                    
                    //// EOF STUDENT ASSIGNMENT
                    
                    // push descriptors for current frame to end of data buffer
                    (dataBuffer.end() - 1)->descriptors = descriptors;
                    
                    
                    cout << "#3 : EXTRACT DESCRIPTORS done" << endl;
                    
                    if (dataBuffer.size() > 1) // wait until at least two images have been processed
                    {
                        
                        /* MATCH KEYPOINT DESCRIPTORS */
                        
                        vector<cv::DMatch> matches;
                        //string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN
                        string descriptorType_2 = "DES_HOG"; // DES_BINARY, DES_HOG
                        string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
                        
                        //// STUDENT ASSIGNMENT
                        //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                        ///
                        //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
                        cout << "descriptors-1 is " << (dataBuffer.end() - 1)->descriptors.size() << endl;
                        cout << "descriptors-2 is " << (dataBuffer.end() - 2)->descriptors.size() << endl;
                        
                        
                        if ( (dataBuffer.end() - 2)->descriptors.empty()||(dataBuffer.end() - 1)->descriptors.empty())
                        {
                            cout << "1st or second descriptor empty"<< endl;
                        }
                        else
                        {
                            
                            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                             matches, descriptorType_2, matcherType, selectorType);
                        }
                        
                        
                        //// EOF STUDENT ASSIGNMENT
                        
                        // store matches in current data frame
                        (dataBuffer.end() - 1)->kptMatches = matches;
                        
                        cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
                        
                        // visualize matches between current and previous image
                        bVis = true;
                        if (bVis)
                        {
                            cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                            cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                            (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                            matches, matchImg,
                                            cv::Scalar::all(-1), cv::Scalar::all(-1),
                                            vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                            
                            string windowName = "Matching keypoints between two camera images";
                            cv::namedWindow(windowName, 7);
                            cv::imshow(windowName, matchImg);
                            cout << "Press key to continue to next image" << endl;
                            //cv::waitKey(0); // wait for key to be pressed
                        }
                        bVis = false;
                        time_log = ((double)cv::getTickCount() - time_log) / cv::getTickFrequency(); // end time counting
                        
                        time_all.push_back(time_log);
                        matchnr_all.push_back(matches.size());
                        keypoint_all.push_back(keypoints.size());
                        det_result_all.push_back(detectorType);
                        des_result_all.push_back(descriptorType);
                        
                    }
                       
                    
                } // eof loop over all images
                std::cout<<"*************************finished*************************" <<endl;
                
                
                //time_log = ((double)cv::getTickCount() - time_log) / cv::getTickFrequency(); // end time counting
                
                //std::cout<<"total time is " << time_log <<endl;
                
                float average_samplelog = 0.0f;
                if ( sample_log.size() != 0)
                {
                    for(int i=0;i<sample_log.size();i++)
                    {
                        average_samplelog +=sample_log[i];
                    }
                    
                    average_samplelog = average_samplelog/sample_log.size();
                }
                std::cout<<"average keypoint number is " << average_samplelog <<endl;
                
                
                //time_all.push_back(time_log);
                //matchnr_all.push_back(average_samplelog);
                //det_result_all.push_back(detectorType);
                //des_result_all.push_back(descriptorType);
                
                cout<<".............................................................."<<endl;
                cout<<".............................................................."<<endl;
                cout<<".............................................................."<<endl;
                cout<<"................................results  ....................."<<endl;

                //std::cout<<"itme i time " << time_log << " sample " <<average_samplelog << " det " << detectorType<< " des " <<descriptorType <<endl;
                //cout<<".............................................................."<<endl;
                //cout<<".............................................................."<<endl;
                
                
            } catch (int e) {
                cout<<"............catch wrong ....................."<<endl;
            }
        }
    }
    for(int i = 0; i <time_all.size();i++)
    {
        std::cout<<"case, time: " << time_all[i] << " match: " <<matchnr_all[i] << "keypoint: " <<keypoint_all[i]<<" det: " << det_result_all[i]<< " des: " <<des_result_all[i] <<endl;
    }
    return 0;
}
