#ifndef _RCAC_HPP_
#define _RCAC_HPP_

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "unsupported/Eigen/KroneckerProduct"
#include <deque>
#include <string>

//Name: Nima Mohseni
//Date: 12/29/2019
//Purpose: This file contains the general implementation of RCAC to support
//derived RCAC classes for RLS, gradient and other RCAC types

/*
struct rcacGradFlags
{
    //RCAC Flags
    int Nc;
    int k_0;
    Eigen::VectorXd theta_0;    
};
*/

/**
 * This struct contains the basic parameters needed for the RCAC filtering to work.
 * 
 * This should be used as a starting point for defining your own Flag structs for
 * different RCAC types.
 * 
 * @param lz Number of performance measurements
 * @param ly Number of sensor measurements
 * @param lu Number of control inputs
 * @param Nc Controller order
 * @param theta_0 Eigen vector with initial controller coefficients. Usually a vector of zeros
 * @param filtorder Order of the filter (\f$G_f\f$)
 */
struct rcacFlags
{
    //Basic RCAC Flags
    int lz;
    int ly;
    int lu;
    int Nc;
    int k_0;
    Eigen::VectorXd theta_0;
    int filtorder;    
};

/**
 * This struct contains the filter coefficients. This struct should not need to 
 * be overloaded unless a big change in the algorithm is needed i.e. all types
 * of RCAC will use this same struct.
 * 
 * @param filtNu Matrix containing the numerator coefficients of (\f$G_f\f$).
 * Should be dimension lz by lu*filtorder
 * @param filtDu Matrix containing the denominator coefficients of (\f$G_f\f$).
 * Dimension lz by lu*(filtorder-1)
 * @param filtNz Matrix containing the numerator coefficients of a filter for the performance measurement.
 * <b> Not currently used in any computations but is required! (set as identity matrix) </b>. 
 * Dimension lz by lz
 * @param filtDz Matrix containing the denominator coefficients of a filter for the performance measurement.
 * <b> Not currently used in any computations but is required! (set as zero matrix) </b>. 
 * Dimension lz by lz
 */
struct rcacFilt
{
    Eigen::MatrixXd filtNu;
    Eigen::MatrixXd filtDu;
    Eigen::MatrixXd filtNz;
    Eigen::MatrixXd filtDz;
};

/**
 * The parent RCAC class. This class handles all the low level computation of RCAC
 * such as the filtering, coefficient updates, and keeping track of the regressors.
 * 
 * Almost all the methods are polymorphic and can be modified by child classes
 * to create RCAC algorithms with more complex filtering.
 */
class RCAC
{
    public:
        /**
         * Factory method for initializing RCAC types.
         * This method is defined in the file RCACCreator.hpp
         * 
         * @param FLAGS template for a struct containing the required flags for 
         * the type of RCAC to be used.
         * @param FILT struct containing the filter coefficients for (\f$G_f\f$).
         * @param whichRCAC string containing the type of RCAC to be used, defined in RCACCreator.hpp.
         */
        template <typename T>
        static RCAC* init(
            T &FLAGS,
            rcacFilt &FILT,
            std::string &whichRCAC
        );

        /*
        //Function initRLS: Initializes RCAC with the given flags and
        //filter values
        void init(
            rcacFlags &FLAGS,
            rcacFilt &FILT
        );
        */

        /*
        //Function initGrad: Initializes RCAC using gradient descent with the 
        //given flags and filter values
        void initGrad(
            rcacGradFlags &FLAGS,
            rcacFilt &FILT
        );
        */

       /**
        * This function allows the user to compute one step of the RCAC update.
        * 
        * @param uIn previous control input
        * @param zIn current performance measurement
        * @param yIn current sensor measurement
        */
        //Function oneStep: Compute one step of RCAC
        void oneStep(
            Eigen::VectorXd &uIn,
            Eigen::VectorXd &zIn,
            Eigen::VectorXd &yIn
        );

        /**
         * Returns RCAC's computed value for the control. Must run oneStep at least once.
         */
        //Function getControl: Get the computed control input
        Eigen::VectorXd getControl()
        {
            return uOut;
        };

        /**
         * Returns a vector of the current RCAC coefficients.
         */
        //Function getCoeff: Get the RCAC coefficients
        Eigen::VectorXd getCoeff()
        {
            return theta;
        };

        /**
         * Returns the number of control inputs that RCAC is using.
         */
        //Function getlu: Get the number of control inputs
        int getlu()
        {
            return lu;
        };

        /**
         * Returns the number of sensor measurements that RCAC is using.
         */
        //Function getly: Get the number of measurements
        int getly()
        {
            return ly;
        };

        /**
         * Returns the number of performance measurements that RCAC is using.
         */
        //Function getlz: Get the number of performance measurements
        int getlz()
        {
            return lz;
        };        

        /**
         * Returns the controller order of RCAC.
         */
        //Function getNc: Get the controller order
        int getNc()
        {
            return Nc;
        };  
        
        /**
         * Returns the timestep of RCAC
         */
        //Function getkk: Get timestep
        int getkk()
        {
            return kk;
        };        

    protected:
        /**
         * Abstract function for the ceofficient update implementation (e.g. RLS, Gradient).
         * Child classes must implement this function for RCAC to work
         * 
         * @param zIn performance measurement
         */
        //Function coeffUpdate: Compute the RCAC coefficient update
        virtual void coeffUpdate(
            Eigen::VectorXd &zIn
        ) = 0;

        /**
         * Initialize the regressor variables uphi and yphi to zero vectors
         */
        //Function initRegressor: initialize the regressor variables
        void initRegressor();

        /**
         * Initialize the variables required for filtering to zero
         */
        //Function initFiltered: initialize filtered variables
        void initFiltered();

        /**
         * Compute the filtered variables given current regressors and filter values
         */
        //Function computeFiltered: compute the filtered variables
        void computeFiltered();

        
        //RCAC Flags
        int lz;
        int ly;
        int lu;
        int Nc;
        int k_0;
        Eigen::VectorXd theta_0;
        int filtorder;


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