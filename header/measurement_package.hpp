//
//  measurement_package.h
//  KalmanFilter
//
//  Created by Bajrang Chapola on 16/05/19.
//  Copyright © 2019 Bajrang Chapola. All rights reserved.
//

#ifndef measurement_package_h
#define measurement_package_h
#include "Eigen/Dense"
class MeasurementPackage{
public:
    enum SensorType{
        LASAR,RADAR
    }sensor_type_;
    Eigen::VectorXd measurements_raw_;
    int64_t timestamp_;
};

#endif /* measurement_package_h */
