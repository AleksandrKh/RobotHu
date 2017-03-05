//
//  PoseEstimator.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 25/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include "PoseEstimator.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include "../Utils/Utils.hpp"

#include <iostream>

using namespace std;
using namespace cv;

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);

#define kCameraID 0
#define kCameraParametersFile "camera.yml" // must be unique for any other camera, see calibrateCamera.cpp
#define kMarkerDictionaryID 8
#define kMarkerID 10
#define kMarkerLengthInMeters 0.1

void PoseEstimator::start() {
        
    startEstimator();
}

void PoseEstimator::startEstimator() {
    
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->doCornerRefinement = true;
    
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(kMarkerDictionaryID));
    
    Mat camMatrix, distCoeffs;
    
    if (!readCameraParameters(kCameraParametersFile, camMatrix, distCoeffs)) {
        
        Utils::printError("Invalid camera calibration camera.yml file. It should be in the same directory as the executed program");
        throw exception();

//        if (didReceiveErrorMessage)
//            (*didReceiveErrorMessage)("Invalid camera file");
        
        return;
    }
    
    VideoCapture inputVideo;
    inputVideo.open(kCameraID);
    
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
        
        if (ids.size() > 0) {
            
            aruco::estimatePoseSingleMarkers(corners, kMarkerLengthInMeters, camMatrix, distCoeffs, rvecs, tvecs);
            
            vector<double> resultVector;
            vector<int> resultIds;
            vector<vector<Point2f> > resultCorners;
            vector<Vec3d> resultRvecs, resultTvecs;
            
            for (unsigned int i = 0; i < ids.size(); i++) {
                
                if (ids[i] != kMarkerID)
                    continue;
                
                // Calc camera pose
                double x = 0, y = 0, z = 0;
                
//                Mat R;
//                Rodrigues(rvecs[i], R);
//                Mat cameraPose = -R.t() * (Mat)tvecs[i];
//
//                x = cameraPose.at<double>(0,0);
//                y = cameraPose.at<double>(0,1);
//                z = cameraPose.at<double>(0,2);
                
                // Rotation invariant (only translation vectors)
                x = tvecs[i][0];
                y = tvecs[i][1];
                z = tvecs[i][2];
                
                //cout << "X: " << tvecs[i][0] << " Y: " << tvecs[i][1] << " Z: " << tvecs[i][2] << endl;
                
                // Take vector from nearest marker
                if (resultVector.empty() || z < resultVector[2]) {
                    
                    resultVector = {x, y, z};
                    
                    resultIds = {ids[i]};
                    resultCorners = {corners[i]};
                    resultRvecs = {rvecs[i]};
                    resultTvecs = {tvecs[i]};
                }
            }
            
            if (resultIds.size()) {
                
                aruco::drawDetectedMarkers(image, resultCorners, resultIds);
                
                aruco::drawAxis(image, camMatrix, distCoeffs, resultRvecs[0], resultTvecs[0], kMarkerLengthInMeters * 0.5f);
                
                if (didObtainPoseDelegate) 
                    (*didObtainPoseDelegate)(resultVector);
            }
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
