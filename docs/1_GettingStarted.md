Getting Started {#GettingStarted}
===============
This is a guide on getting started with the RCAC library.

How to install?
===============
First download the source code from the [repository](https://github.com/mohseninima/RCAC_cpp).
Only the *.cpp and *.hpp files are needed. Next, you will need a copy of the Eigen matrix library,
located [here](http://eigen.tuxfamily.org/). Extract the *Eigen* and *unsupported* folders and place them in the same folder as the RCAC code.
You should have a folder structure similar to the following.

~~~~~~~~~~~~~~~~~~~
<root>
    Eigen/
    unsupported/
    RCAC.cpp
    RCAC.hpp
    RCACCreator.hpp
    RCAC*.cpp
    RCAC*.hpp
    .
    .
    .
    main.cpp (your code)
~~~~~~~~~~~~~~~~~~~

Using Recursive Least Squares (RLS) RCAC in C++
=========================================
Using RLS RCAC in your own code requires 6 main steps

1. Including "RCACCreator.hpp" and Eigen
2. Defining and assigning the rcacRlsFlags struct
3. Defining and assigning the rcacFilt struct
4. Initializing RCAC using RCAC::init()
5. Using the RCAC::oneStep() method to propagate RCAC by one step
6. Using the RCAC::getControl() method to get the associated control input

Creating the rcacRlsFlags struct
--------------------------------
This struct defines the base parameters such as controller order and sizes of the inputs and outputs.
The list of members is
* lu: Number of control inputs (Type: int)
* ly: Number of sensor measurements (Type: int)
* lz: Number of performance measurements (Type: int)
* Nc: Controller order (Type: int)
* filtorder: Order of the filter \f$G_f\f$ (Type: int)
* k_0: Starting timestep for the controller (Type: int) (Usually = Nc)
* P0: Initial value of the covariance matrix for RLS (Type: Eigen::MatrixXd) (ltheta by ltheta)
* Ru: Weighting matrix on the control input (Type: Eigen::MatrixXd) (lu by lu)
* Rz: Weighting matrix on the retrospective performance (usually identity) (Type: Eigen::MatrixXd) (lz by lz)
* lambda: The forgetting factor for RLS (Type: double)
* theta_0: Initial value for the controller coefficients (Type: Eigen::VectorXd) (ltheta by 1)

The parameter ltheta is not needed to be given to RCAC directly but is useful for initialization. It is defined as
\f$ l_\theta = N_c l_u(l_u+l_y) \f$

An example struct for a 2 input 2 output plant with a 10th order controller and filtorder of 4 is given below
~~~~~~~~~~~~~~~~~~~{.cpp}
rcacRlsFlags FLAGS;
FLAGS.lu = 2;
FLAGS.lz = 2;
FLAGS.ly = 2;
FLAGS.Nc = 4;
int ltheta = FLAGS.Nc*FLAGS.lu*(FLAGS.lu+FLAGS.ly); //Define ltheta to help create other variables
FLAGS.filtorder = 4;
FLAGS.k_0 = FLAGS.Nc;
FLAGS.P0 = 100000*Eigen::MatrixXd::Identity(ltheta, ltheta); //Create a matrix with diag values of 100000
FLAGS.Ru = Eigen::MatrixXd::Zero(FLAGS.lu, FLAGS.lu); //Create a zero matrix (no weighting on control effort)
FLAGS.Rz = Eigen::MatrixXd::Identity(FLAGS.lz, FLAGS.lz); //Create the identity matrix
FLAGS.lambda = 1;
FLAGS.theta_0 = Eigen::MatrixXd::Zero(ltheta, 1); //Initialize the RCAC coefficients to zero
~~~~~~~~~~~~~~~~~~~


Creating the rcacFilt struct
--------------------------------
This struct defines the coefficients of the RCAC filter \f$G_f\f$ and a filter for the performance measurements
The list of members is
* filtNu: Numerator coefficients of \f$G_f\f$ (Type: Eigen::MatrixXd) (lz by lu*filtorder)
* filtDu: Denominator coefficients of \f$G_f\f$ (Type: Eigen::MatrixXd) (lz by lu*(filtorder-1))
* filtNz: Numerator coefficients of a filter on the performance measurement. Usually identity. (Type: Eigen::MatrixXd) (lz by lz)
* filtDz: Denominator coefficients of a filter on the performance measurement. Usually a zero matrix. (Type: Eigen::MatrixXd) (lz by lz)

An example struct for a 2 input 2 output plant with a filterorder of 4 is given below. In this example, the comma initializer
syntax from Eigen is used to construct filtNu and filtDu
~~~~~~~~~~~~~~~~~~~{.cpp}
rcacFilt FILT;
FILT.filtNu.resize(FLAGS.lz, FLAGS.lu*filtorder); //Set the size of the matrix to use the comma initializer syntax
FILT.filtNu << 0, 0, 1, 0, 0.25, 1, 0.0625, 0.5, //Use the comma initializer syntax
               0, 0, 0, 3, 0, 0.75, 0, 0.1875;

FILT.filtDu.resize(FLAGS.lz, FLAGS.lz*(filtorder-1));
FILT.filtDu << 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0;

FILT.filtNz = MatrixXd::Identity(FLAGS.lz, FLAGS.lz);
FILT.filtDz =  MatrixXd::Zero(FLAGS.lz, FLAGS.lz);
~~~~~~~~~~~~~~~~~~~


Initializing RCAC
-----------------
Once the required rcacRlsFlags and rcacFilt structs are created. RCAC can be initialized.
Initialization requires a flag struct, filt struct, and a string specifying the type of RCAC to be used.

~~~~~~~~~~~~~~~~~~~{.cpp}
//String stating the type of RCAC to use
std::string rcacType = "RLS";

//Create a pointer to an RCAC object
RCAC *myRCAC;

//initialize RCAC with flags defined by an rcacRlsFlags struct
myRCAC = myRCAC->init<rcacRlsFlags>(FLAGS, FILT, rcacType);
~~~~~~~~~~~~~~~~~~~

Compiling
-----------------
A program that uses RCAC can be compiled using the following command
~~~~~~~~~~~~~~~~~
g++ -O3 main.cpp RCAC.cpp RCACRLS.cpp RCACGrad.cpp -o main
~~~~~~~~~~~~~~~~~

Putting it all together
------------------------
Below is an example of using RCAC to control a 2x2 2nd order MIMO system to follow a step command with a 4th order controller using 3 markov parameters

~~~~~~~~~~~~~~~~~~~{.cpp}
#include "Eigen/Core"
#include "RCACCreator.hpp"
#include <iostream>
#include <string>
using namespace Eigen;

int main()
{
    //**************Simulation Parameters***************
    //Initialize the plant
    int lx = 2;
    int lu = 2;
    int ly = 2;
    int lz = ly;

    MatrixXd A(lx,lx);
    A << 0.25,1,
          0,0.25;

    MatrixXd B(lx,lu);
    B << 1,0,
         0,1;

    MatrixXd C(ly,lx);
    C << 1,0,
         0,3;

    MatrixXd x(lx,1);
    x << 0,
         0;

    //End time of the simulation
    int kend = 30;
    //*************************************************


    //***************RCAC Parameters*******************
    //Set Flags
    rcacRlsFlags FLAGS;
    FLAGS.lu = lu;
    FLAGS.lz = lz;
    FLAGS.ly = ly;
    FLAGS.Nc = 4;
    int ltheta = FLAGS.Nc*FLAGS.lu*(FLAGS.lu+FLAGS.ly);
    FLAGS.filtorder = 4;
    FLAGS.k_0 = FLAGS.Nc;
    FLAGS.P0 = 100000*MatrixXd::Identity(ltheta, ltheta);
    FLAGS.Ru = MatrixXd::Zero(FLAGS.lu, FLAGS.lu);
    FLAGS.Rz = MatrixXd::Identity(FLAGS.lz, FLAGS.lz);
    FLAGS.lambda = 1;
    FLAGS.theta_0 = MatrixXd::Zero(ltheta, 1);

    std::cout << "Flags Set\n"; 

    //Set Filter
    rcacFilt FILT;
    int Hnum = FLAGS.filtorder-1;
    FILT.filtNu.resize(FLAGS.lz, FLAGS.lu*(Hnum+1));
    FILT.filtDu.resize(FLAGS.lz, FLAGS.lz*Hnum);
    FILT.filtNu << 0, 0, 1, 0, 0.25, 1, 0.0625, 0.5,
                   0, 0, 0, 3, 0, 0.75, 0, 0.1875; 
    FILT.filtDu << 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0;
    FILT.filtNz = MatrixXd::Identity(FLAGS.lz, FLAGS.lz);
    FILT.filtDz =  MatrixXd::Zero(FLAGS.lz, FLAGS.lz);

    std::cout << "Filter Set\n"; 

    //Initialize RCAC
    std::string rcacType = "RLS";
    RCAC *myRCAC;
    //Call the factory method to initialize RCAC
    myRCAC = myRCAC->init<rcacRlsFlags>(FLAGS, FILT, rcacType);

    std::cout << "RLS Init\n"; 
    //**************************************************


    //******************Simulation**********************
    //Run Simulation
    VectorXd y(FLAGS.ly,1);
    VectorXd z(FLAGS.lz,1);
    VectorXd u(FLAGS.lu,1);
    u = MatrixXd::Zero(FLAGS.lu, 1);
    
    for (int k = 0; k < kend; k++)
    {
        y = C*x;
        z = y - VectorXd::Ones(FLAGS.ly);

        x = A*x + B*u;        

        myRCAC->oneStep(u, z, z); //Compute one step of RCAC
        u = myRCAC->getControl(); //Get the resulting control

        std::cout << "y: " << y << ", z: " << z << ", u: " << u << std::endl;
    }
    //****************************************************

    return(0);
}
~~~~~~~~~~~~~~~~~~~