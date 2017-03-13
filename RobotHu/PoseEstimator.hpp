//
//  PoseEstimator.hpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 25/02/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#ifndef PoseEstimator_hpp
#define PoseEstimator_hpp

#include <vector>
#include <functional>

struct PoseVector { // it's Y invariant pose, also we consider marker rotation in XZ plane
    
    double xzAngleInDeg;
    double xDistanceInMeters;
    double zDistanceInMeters;
};

class PoseEstimator {
    
public:
    
    static PoseEstimator& Instance() {
        
        static PoseEstimator s;
        return s;
    }
    
    void start();
    
    // Delegates
    std::function<void(PoseVector)> *didObtainPoseDelegate;
    std::function<void(std::string errorMessage)> *didReceiveErrorMessage;
    
private:
    
    PoseEstimator() {}
    ~PoseEstimator() {}
    
    PoseEstimator(PoseEstimator const&) = delete;
    PoseEstimator& operator = (PoseEstimator const&) = delete;
    
    void startEstimator();
};

#endif /* PoseEstimator_hpp */
