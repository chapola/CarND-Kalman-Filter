//
//  kalman_filter.hpp
//  KalmanFilter
//
//  Created by Bajrang Chapola on 16/05/19.
//  Copyright © 2019 Bajrang Chapola. All rights reserved.
//

#ifndef kalman_filter_hpp
#define kalman_filter_hpp
#include "Eigen/Dense"
#include <stdio.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
class KalmanFilter{
  
public:
    // Constructor
    KalmanFilter();
    
    // Distructor
    virtual ~KalmanFilter();
    
    // Pridict the state and state covariance using process model
    void predict();
    
    // update the state and measurement at time k+1
    
    void update(const Eigen::VectorXd &z);
        
        // state vector
        VectorXd x_;
        
        // state covariance matrix P
        MatrixXd P_;
        
        //state transition matrix F
        
        MatrixXd F_;
        
        //Process Covariance matrix Q
        
        MatrixXd Q_;
        
        // measurement matrix H
        
        MatrixXd H_;
        
        // measurement covariance matrix R
        MatrixXd R_;
    
    
    
};

#endif /* kalman_filter_hpp */
