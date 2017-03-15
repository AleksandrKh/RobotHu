//
//  PoseEstimator.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 25/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include "PoseEstimator.hpp"
#include "config.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include "../Utils/Utils.hpp"
#include <iostream>

using namespace std;
using namespace cv;

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);

void PoseEstimator::start() {
        
    startEstimator();
}

void PoseEstimator::startEstimator() {
    
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->doCornerRefinement = true;
    
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(kMarkerDictionaryID));
    
    Mat camMatrix, distCoeffs;
    
    if (!readCameraParameters(kCameraParametersFile, camMatrix, distCoeffs)) {
        
        Utils::printError("Invalid camera calibration .yml file. It should be in the same directory as the executed program");

//        if (didReceiveErrorMessage)
//            (*didReceiveErrorMessage)("Invalid camera file");
        
        throw exception();
    }
    
    VideoCapture inputVideo;
    inputVideo.open(kCameraID);
    
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, kCameraFrameWidth);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, kCameraFrameHeight);

    if (!inputVideo.isOpened()) {
        Utils::printError("Camera is not connected");
        throw exception();
    }
    
    while (inputVideo.grab()) {
        
        Mat image;
        
        inputVideo.retrieve(image);
        
        vector<int> ids;
        vector<vector<Point2f> > corners;
        vector<Vec3d> rvecs, tvecs;
        
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams);
        
        bool poseFound = false;
        
        if (ids.size() > 0) {
            
            aruco::estimatePoseSingleMarkers(corners, kMarkerLengthInMeters, camMatrix, distCoeffs, rvecs, tvecs);
            
            PoseVector resultPose = {0, 0, 0};
            vector<int> resultIds;
//            vector<vector<Point2f> > resultCorners;
//            vector<Vec3d> resultRvecs, resultTvecs;
            
            for (unsigned int i = 0; i < ids.size(); i++) {
                
                if (ids[i] != kMarkerID)
                    continue;
                
                double xyRot = atan(rvecs[i][1] / rvecs[i][0]);
                
                // check Y axis direction to use angles properly // TODO: make invariant
                if (fabs(RADIANS_TO_DEGREES(xyRot)) > kMaxMarkerXYRotDeviationInDegrees)
                    continue;
                
                // Calc camera pose
                double x = 0, y = 0, z = 0;
                
                Mat R;
                Rodrigues(rvecs[i], R);
                Mat cameraPose = -R.t() * (Mat)tvecs[i];
                
                x = cameraPose.at<double>(0,0);
                y = cameraPose.at<double>(0,1);
                z = cameraPose.at<double>(0,2);
                                
                // cout << "X: " << x << " Y: " << y << " Z: " << z << endl;
                
                // Take vector from nearest marker
                if (resultPose.zDistanceInMeters == 0 || z < resultPose.zDistanceInMeters) {
                    
                    resultPose.zDistanceInMeters = z;
                    resultPose.xDistanceInMeters = tvecs[i][0]; // rot invariant
                    
                    // Rotation in XZ plane // TODO: make invariant
                    resultPose.xzAngleInDeg = RADIANS_TO_DEGREES(atan2(sqrt(pow(x, 2) + pow(y, 2)), z));
                    resultPose.xzAngleInDeg *= x / fabs(x);
                    
                    resultIds = {ids[i]};
//                    resultCorners = {corners[i]};
//                    resultRvecs = {rvecs[i]};
//                    resultTvecs = {tvecs[i]};
                }
            }
            
            if (resultIds.size()) {
                
                // aruco::drawDetectedMarkers(image, resultCorners, resultIds);
                
                // aruco::drawAxis(image, camMatrix, distCoeffs, resultRvecs[0], resultTvecs[0], kMarkerLengthInMeters * 0.5f);
                
                poseFound = true;
                
                if (didObtainPoseDelegate) 
                    (*didObtainPoseDelegate)(resultPose);
            }
        }
        
        if (!poseFound) {
            
            if (didLostPoseDelegate)
                (*didLostPoseDelegate)();
        }
        
        // Only from main thread
//        resize(image, image, Size(image.cols / 4, image.rows / 4));
//        imshow("out", image);
//        char key = (char)waitKey(kWaitTimeInmS);
//        if (key == 27) break;
    }
}

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}
