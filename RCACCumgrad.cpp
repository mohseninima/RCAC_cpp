#include "RCACCumgrad.hpp"
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Sparse"
#include "unsupported/Eigen/KroneckerProduct"

//Name: Nima Mohseni
//Date: 4/5/2020
//Purpose: This file contains the implementation of Cumulative Gradient Descent 
//RCAC as a derived class of the base RCAC class

RCACCumgrad::RCACCumgrad(
    rcacCumgradFlags &FLAGS,
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
    gamma = FLAGS.gamma;
    lambda = FLAGS.lambda;

    this->FILT = FILT;

    //Initialize the regressor variables
    initRegressor();

    //Initialized Filtered Variables
    initFiltered();

    //initialize the step
    kk = 1;

    //initialize the gradient to a zero vector
    gradient.Zero(Nc*lu*(lu+ly));

    //Initialize the Asum matrix to a zero matrix
    Asum.Zero(Nc*lu*(lu+ly),Nc*lu*(lu+ly));

    //Initialize the bsum vector to a zero vector
    bsum.Zero(Nc*lu*(lu+ly));
}

void RCACCumgrad::coeffUpdate(
    Eigen::VectorXd &zIn
)
{
    double stepSize;

    //Change the computation of the gradient/stepsize for different forgetting factors
    if (lambda == 1)
    {
        Asum.noalias() += (PhifBar[0].transpose()*PhifBar[0]);
        bsum.noalias() += (PhifBar[0].transpose())*(zIn - ufBar[0]);
    }
    else if (lambda != 0) //only compute if forgetting is not 0
    {
        Asum = (PhifBar[0].transpose()*PhifBar[0]) + lambda*Asum;
        bsum = (PhifBar[0].transpose())*(zIn - ufBar[0]) + lambda*bsum;
    }
    
    //Use different computation of the gradient for different regularization values
    if (gamma > 0) //Cumgrad with regularization
    {
        stepSize = alpha/(Asum.norm()); //ignoring regularization in the stepsize update
        gradient = Asum*theta + bsum + gamma*theta;
    }
    else if (lambda == 0 && gamma > 0) //Just regular gradient with regularization
    {
        stepSize = alpha/pow(PhifBar[0].norm(),2); //ignoring regularization in the stepsize update
        gradient = PhifBar[0].transpose()*(zIn - ufBar[0] + PhifBar[0]*theta)
                   + gamma*theta;
    }
    else if (lambda == 0)
    {
        stepSize = alpha/pow(PhifBar[0].norm(),2);
        gradient = PhifBar[0].transpose()*(zIn - ufBar[0] + PhifBar[0]*theta);
    }
    else //Cumgrad with no regularization
    {
        stepSize = alpha/(Asum.norm());
        gradient = Asum*theta + bsum;
    }

    //update the coefficients
    theta.noalias() += -stepSize*gradient;
 
    //std::cout << "kk: " << kk << ", " << theta.transpose() << "\n";
}