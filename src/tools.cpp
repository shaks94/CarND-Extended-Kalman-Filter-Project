#include <iostream>
#include "tools.h"
#include<math.h>
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    long estimations_size =estimations.size() ;
    // Check the validity of the following inputs:
    // The estimation vector size should not be zero
    if(estimations_size == 0){
        cout << "Input is empty" << endl;
        return rmse;
    }
    // The estimation vector size should equal ground truth vector size
    if(estimations_size != ground_truth.size()){
        cout << "Invalid estimation or ground_truth. Data should have the same size" << endl;
        return rmse;
    }
    // Accumulate squared residuals
    for(unsigned int i=0; i < estimations_size; ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        // Coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }
    
    // Calculate the mean
    rmse = rmse / estimations_size;
    rmse = rmse.array().sqrt();
    cout<<"rms"<<rmse<<endl;
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3,4);

    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    if(fabs(c1) < .00001) {
        cout<<"iin for c1";
        px += .001;
        py += .001;
        c1 = px * px + py * py;
    }
    float c2 = sqrt(c1);
    float c3 = (c1*c2);
    if(fabs(c1) < 0.0001){
        cout << "ERROR - CalculateJacobian () - Division by Zero" << endl;
        return Hj;
    }    //check division by zero

    
    //compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
    -(py/c1), (px/c1), 0, 0,
    py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    
    return Hj;

}
