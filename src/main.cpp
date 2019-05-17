//
//  main.cpp
//  KalmanFilter
//
//  Created by Bajrang Chapola on 16/05/19.
//  Copyright © 2019 Bajrang Chapola. All rights reserved.
//

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.hpp"
#include "tracking.hpp"
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
int main(int argc, const char * argv[]) {
    // insert code here...
    // Set measurements in a list
    vector<MeasurementPackage> measurements_list;
    // hardcoded input file with laser and radar measurements
    string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
    
    ifstream in_file(in_file_name_.c_str(),ifstream::in);
    if (!in_file.is_open()) {
        cout<<"Can not open input file"<<in_file_name_<<endl;
    }
    
    string line;
    int i=0; // number of the lines we want to read
    while (getline(in_file, line)) {
        MeasurementPackage measurement_package_;
        
        istringstream iss(line);
        int64_t timestamp;
        string sensor_type;
        // Read the first element of line
        iss>>sensor_type;
        cout<<"Sensor type:::"<<sensor_type<<endl;
        if (sensor_type.compare("L")==0) {
            measurement_package_.sensor_type_=MeasurementPackage::LASAR;
            measurement_package_.measurements_raw_=VectorXd(2);
            float x;
            float y;
            iss>>x;
            iss>>y;
            iss>>timestamp;
            measurement_package_.measurements_raw_<<x,y;
            measurement_package_.timestamp_=timestamp;
            measurements_list.push_back(measurement_package_);
            
        }else if (sensor_type.compare("R")){
            continue;
        }
       
        i++;
    }
   // Create a Tracking instance
    Tracking tracking;
    
    //size of the measurements list
    size_t N=measurements_list.size();
    cout<<"Size of the list :::"<<N<<endl;
    
    // start filtering from the 2nd frame because speed in unknown in 1st measurement
    
    for (size_t k=0; k<N; ++k) {
        tracking.ProcessMeasurement(measurements_list[k]);
    }
    // close the file
    if (in_file.is_open()) {
        in_file.close();
    }
    return 0;
}
