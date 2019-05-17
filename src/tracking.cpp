//
//  tracking.cpp
//  KalmanFilter
//
//  Created by Bajrang Chapola on 17/05/19.
//  Copyright © 2019 Bajrang Chapola. All rights reserved.
//

#include "tracking.hpp"
#include <iostream>
Tracking::Tracking(){
    is_initialized_=false;
    previous_timestamp_=0;
    
    // create a 4-D state vector, we don't know the value of x yet
    kf_.x_=VectorXd(4);
    
    kf_.P_=MatrixXd(4,4);
    kf_.P_<<1,0,0,0,
            0,1,0,0,
            0,0,1000,0,
            0,0,0,1000;
    
    // measurement covariance R
    kf_.R_=MatrixXd(2,2);
    kf_.R_<<0.0225,0,
            0,0.0225;
    
    // measurement matrix
    kf_.H_=MatrixXd(2,4);
    kf_.H_<<1,0,0,0,
            0,1,0,0;
    
    // initial transition matrix F
    kf_.F_=MatrixXd(4,4);
    kf_.F_<<1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
    
    noise_ax=5;
    noise_ay=5;
    
    
    
}
Tracking::~Tracking(){
    
}
/**
 * process the single measurement
 */
void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_package){
    if (!is_initialized_) {
        kf_.x_<<measurement_package.measurements_raw_[0],
                measurement_package.measurements_raw_[1],
                0,
                0;
        is_initialized_=true;
        previous_timestamp_=measurement_package.timestamp_;
        return;
    }
   
    //compute the time elapsed between current measurement and the previous measurement
    float delta_t = measurement_package.timestamp_ - previous_timestamp_;
    previous_timestamp_=measurement_package.timestamp_;
    
    // Modify the F Matrix
    
    kf_.F_ <<1, 0, delta_t, 0,
            0, 1, 0, delta_t,
            0, 0, 1, 0,
            0, 0, 0, 1;
    
    //set the process covariance matrix Q
    float dt_2 = delta_t*delta_t;
    float dt_3 = dt_2*delta_t;
    float dt_4 = dt_3*delta_t;
    kf_.Q_=MatrixXd(4,4);
    kf_.Q_<<dt_4/4*noise_ax,0,dt_3/2*noise_ax,0,
            0,dt_4/4*noise_ay,0,dt_3/2*noise_ay,
            dt_3/2*noise_ax,0,dt_2*noise_ax,0,
            0,dt_3/2*noise_ay,0,dt_2*noise_ay;
    //Call the Kalman Filter predict() function
    kf_.predict();
    //Call the Kalman Filter update() function with the most recent raw measurements_
    
    kf_.update(measurement_package.measurements_raw_);
    
    cout << "x_= " << kf_.x_ << endl;
    cout << "P_= " << kf_.P_ << endl;
    
}
