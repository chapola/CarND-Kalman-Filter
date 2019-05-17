//
//  kalman_filter.cpp
//  KalmanFilter
//
//  Created by Bajrang Chapola on 16/05/19.
//  Copyright © 2019 Bajrang Chapola. All rights reserved.
//

#include "kalman_filter.hpp"
#include <iostream>
KalmanFilter::KalmanFilter(){
    
}
KalmanFilter::~KalmanFilter(){
    
}
/**
 * @predict funtion Predict the new value of state, based on the intial value and then
 * predcit the uncertanity/error/covariance in our prediction according to the various
 * process noise.
 */
void KalmanFilter::predict(){
    x_=F_*x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_*P_*Ft+Q_;
   
    
};
/**
 Here we calculte the differance between predicted value and the measured value, then decide which
  value to keep based on the Kalman Gain. Again based on our decison we calculte the  uncertanity/error/covariance matrix.
 */
void KalmanFilter::update(const Eigen::VectorXd &z){
    VectorXd y = z-H_*x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_*P_*Ht+R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt= P_*Ht;
    MatrixXd K=PHt*Si;
    
    // new state
    
    x_ = x_+(K*y);
    long x_size=x_.size();
    MatrixXd I = MatrixXd::Identity(x_size,x_size);
    P_=(I-K*H_)*P_;
    
    
}

