#ifndef _RCAC_HPP_
#define _RCAC_HPP_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/KroneckerProduct>
#include <deque>

//Name: Nima Mohseni
//Date: 12/29/2019
//Purpose: This file contains the implementation of RCAC for both RLS and gradient
//implementations in the form of a portable library

struct rcacRLSFlags
{
    //RCAC Flags
    int lz;
    int ly;
    int lu;
    int Nc;
    Eigen::MatrixXd Rtheta;
    Eigen::MatrixXd Ru;
    Eigen::MatrixXd Rz;
    int k_0;
    int lambda;
    Eigen::VectorXd theta_0;    
};

/*
struct rcacGradFlags
{
    //RCAC Flags
    int Nc;
    int k_0;
    Eigen::VectorXd theta_0;    
};
*/

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

        /*
        //Function initGrad: Initializes RCAC using gradient descent with the 
        //given flags and filter values
        void initGrad(
            rcacGradFlags &FLAGS,
            rcacFilt &FILT
        );
        */

        //Function oneStep: Compute one step of RCAC
        void oneStep(
            Eigen::VectorXd &uIn,
            Eigen::VectorXd &zIn,
            Eigen::VectorXd &yIn
        );

        //Function getControl: Get the computed control input
        Eigen::VectorXd getControl()
        {
            return uOut;
        };

    private:
        //Function coeffUpdate: Compute the RCAC coefficient update
        void coeffUpdate(
            Eigen::VectorXd &zIn
        );

        //Function initRegressor: initialize the regressor variables
        void initRegressor();

        //Function initFiltered: initialize filtered variables
        void initFiltered();

        //Function computeFiltered: compute the filtered variables
        void computeFiltered();

        
        //RCAC Flags
        int lz;
        int ly;
        int lu;
        int Nc;
        int k_0;
        int lambda;

        //RLS FLAGS
        Eigen::VectorXd theta_0;
        Eigen::MatrixXd Rtheta;
        Eigen::MatrixXd Ru;
        Eigen::MatrixXd Rz;


        //rcacGradFlags gradFLAGS;

        //RCAC Filter
        /*
        Eigen::MatrixXd filtNu;
        Eigen::MatrixXd filtDu;
        Eigen::MatrixXd filtNz;
        Eigen::MatrixXd filtDz;
        */

        rcacFilt FILT;

        //RCAC Filtered Variables
        std::deque<Eigen::VectorXd> uBar;
        std::deque<Eigen::VectorXd> ufBar;
        std::deque<Eigen::VectorXd> zBar;
        std::deque<Eigen::VectorXd> zfBar;
        //std::deque<Eigen::MatrixXd> uPhiBar;
        //std::deque<Eigen::MatrixXd> yPhiBar;
        std::deque<Eigen::MatrixXd> PhiBar;
        std::deque<Eigen::MatrixXd> PhifBar;  

        //RCAC Working variables
        Eigen::MatrixXd P; 
        Eigen::VectorXd theta;
        Eigen::VectorXd uOut;
        Eigen::VectorXd uIn;
        Eigen::MatrixXd Phi; //kron([uphi;yphi]', eye_lu)
        Eigen::VectorXd uphi; //Past Nc u values
        Eigen::VectorXd yphi; //Past Nc y values

        //RCAC Types: tell the algorithm what type of RCAC to use
        bool rcacRLS;
        bool rcacGrad;

        //Counter: if kk >= k_0, start RCAC control input
        int kk = 1;
};

#endif