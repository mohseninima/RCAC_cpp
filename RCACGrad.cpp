#include "RCACGrad.hpp"
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Sparse"
#include "unsupported/Eigen/KroneckerProduct"

//Name: Nima Mohseni
//Date: 2/18/2020
//Purpose: This file contains the implementation of Gradient Descent RCAC as
//a derived class of the base RCAC class

RCACGrad::RCACGrad(
    rcacGradFlags &FLAGS,
    rcacFilt &FILT
)
{
    //Assign FLAGS and FILT variables to the specific RCAC object
    lz = FLAGS.lz;
    ly = FLAGS.ly;
    lu = FLAGS.lu;
    Nc = FLAGS.Nc;
    theta_0 = FLAGS.theta_0;
    filtorder = FLAGS.filtorder;
    k_0 = FLAGS.k_0;
    alpha = FLAGS.alpha;

    this->FILT = FILT;

    //Initialize the regressor variables
    initRegressor();

    //Initialized Filtered Variables
    initFiltered();

    //initialize the step
    kk = 1;

    //initialize the gradient to a zero vector
    gradient.Zero(Nc*lu*(lu+lu));

}

void RCACGrad::coeffUpdate(
    Eigen::VectorXd &zIn
)
{
    //compute the step size and gradient
    double stepSize = alpha/pow(PhifBar[0].norm(),2);
    gradient = PhifBar[0].transpose()*(zIn - ufBar[0] + PhifBar[0]*theta);

    //update the coefficients
    theta = theta - stepSize*gradient;
 
    //std::cout << "kk: " << kk << ", " << theta.transpose() << "\n";
}