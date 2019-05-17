//
//  tracking.hpp
//  KalmanFilter
//
//  Created by Bajrang Chapola on 17/05/19.
//  Copyright © 2019 Bajrang Chapola. All rights reserved.
//

#ifndef tracking_hpp
#define tracking_hpp
#include "measurement_package.hpp"
#include "kalman_filter.hpp"
using namespace std;
class Tracking{
public:
    Tracking();
    virtual ~Tracking();
    void ProcessMeasurement(const MeasurementPackage &measurement_package);
    KalmanFilter kf_;
private:
    bool is_initialized_;
    int64_t previous_timestamp_;
    
    // accelerating noise
    
    float noise_ax=5;
    float noise_ay=5;
    
};
#endif /* tracking_hpp */
