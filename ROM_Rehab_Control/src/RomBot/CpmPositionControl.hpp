#ifndef CPM_POSITION_CONTROL_H
#define CPM_POSITION_CONTROL_H
//#include <Arduino.h> 
#include <math.h> 
#include "RomBot.hpp" 
//! A class for Position-based impedence controller for a RP rehabilitation robot
/*!
   This class contains all the necessary computations needed to find the desired control position at the two motors given impedence parameters. There are a number of functions that can 
   interact with the CpmPositionControl object to get current xc values and to update the xc values as the RomBot object evloves over time (i.e. sensor values change).  
*/
class CpmPositionControl { 

    public: 

        /**
        * @brief Constructor for CpmPositionControl Class
        * @param[in] M_i Inertial impedence parameters 
        * @param[in] B_i Dampening impedence parameters 
        * @param[in] K_i Spring impedence parameters 
        * @param[in] sampleTime Sample time for control algorithm   
        */
        CpmPositionControl(double M_i[],double B_i[],double K_i[], double sampleTime);


        /**
        * @brief Function to update position impedence error values from RomBot object and a waypoint  
        * @param[in] robot RomBot object instance that contains robot parameter information 
        * @param[in] mjtWaypoint Array of doubles read from Serial that contains desired kinematic terms 
        */
        void updateImpedencePosition(RomBot& robot, double mjtWaypoint[]);

        
        /**
        * @brief Getter for revolute joint desired control position  
        * @return desired control position at revolute joint taken directly from xc_[0]
        */
        double getThetaPos(); 

        /**
        * @brief Getter for revolute joint desired control position  
        * @return desired control position at prismatic joint taken directly from xc_[1]
        */
        double getRPos(); 
        
        /**
        * @brief Getter for Position Error  
        * @return Array of position error terms at the EE
        */
        double *getThetaError(); 

        /**
        * @brief Getter for Velocity Error  
        * @return Array of velocity error terms at the EE
        */
        double *getRError();
        
        /// Inertial Impedence Term 
        double M_i[2]; 

        /// Damping Impedence Term
        double B_i[2];

        /// Spring-stiffness Impedence Term 
        double K_i[2]; 

    private:
        
        /// Position error array that contains error[n - 1] and error[n - 2] respectively 
        double errorTheta_[2] = {0,0};

        /// Position error array that contains error[n - 1] and error[n - 2] respectively  
        double errorR_[2] = {0,0};   

        /// Output Positions  
        double xc_[2] = {0,0}; 

        double ts_; 

};
#endif // CPM_POSITION_CONTROL_H