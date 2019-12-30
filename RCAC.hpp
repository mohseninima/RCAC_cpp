#ifndef _RCAC_HPP_
#define _RCAC_HPP_

#include <Eigen/Dense>
#include <deque>

//Name: Nima Mohseni
//Date: 12/29/2019
//Purpose: This file contains the implementation of RCAC for both RLS and gradient
//implementations in the form of a portable library

struct rcacRLSFlags
{
    //RCAC Flags
    int Nc;
    Eigen::MatrixXd Rtheta;
    Eigen::MatrixXd Ru;
    Eigen::MatrixXd Rz;
    int k_0;
    int lambda;
    Eigen::VectorXd theta_0;    
};

struct rcacGradFlags
{
    //RCAC Flags
    int Nc;
    int k_0;
    Eigen::VectorXd theta_0;    
};

struct rcacFilt
{
    Eigen::MatrixXd filtNu;
    Eigen::MatrixXd filtDu;
    Eigen::MatrixXd filtNz;
    Eigen::MatrixXd filtDz;   
};

class RCAC
{
    public:
        //Function initRLS: Initializes RCAC using RLS with the given flags and
        //filter values
        void initRLS(
            rcacRLSFlags &FLAGS,
            rcacFilt &FILT
        );

        //Function initGrad: Initializes RCAC using gradient descent with the 
        //given flags and filter values
        void initGrad(
            rcacGradFlags &FLAGS,
            rcacFilt &FILT
        );

        //Function oneStep: Compute one step of RCAC
        void oneStep(
            Eigen::VectorXd &uIn,
            Eigen::VectorXd &zIn,
            Eigen::VectorXd &yIn
        );

        //Function getControl: Get the computed control input
    
    private:
        /*
        //RCAC Flags
        int Nc;
        Eigen::MatrixXd Rtheta;
        Eigen::MatrixXd Ru;
        Eigen::MatrixXd Rz;
        int k_0;
        int lambda;
        Eigen::VectorXd theta_0;
        */
        rcacRLSFlags rlsFLAGS;
        rcacGradFlags gradFLAGS;

        //RCAC Filter
        /*
        Eigen::MatrixXd filtNu;
        Eigen::MatrixXd filtDu;
        Eigen::MatrixXd filtNz;
        Eigen::MatrixXd filtDz;
        */

        rcacFilt FILT;

        //RCAC Filtered Variables
        std::deque<Eigen::VectorXd> ubar;
        std::deque<Eigen::VectorXd> ufbar;
        std::deque<Eigen::VectorXd> zbar;
        std::deque<Eigen::VectorXd> zfbar;
        std::deque<Eigen::MatrixXd> Phibar;
        std::deque<Eigen::MatrixXd> Phifbar;  

        //RCAC Working variables
        Eigen::MatrixXd P;
        Eigen::VectorXd theta;
        Eigen::VectorXd uOut;
        Eigen::VectorXd uIn;

        //RCAC Types: tell the algorithm what type of RCAC to use
        bool rcacRLS;
        bool rcacGrad;
};

#endif