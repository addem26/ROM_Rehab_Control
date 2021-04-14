#ifndef CPM_CONTROLLER_H
#define CPM_CONTROLLER_H 
//#include <Arduino.h> 
#include <math.h> 
#include "RomBot.hpp" 
//! A class for Position-based impedence controller for a RP rehabilitation robot
/*!
   This class contains all the necessary computations needed to find the desired torque at the two motors given impedence parameters. There are a number of functions that can 
   interact with the RomBot objects to get current torque values and to update the torque values as the RomBot object evloves over time (i.e. sensor values change).  
*/
class CpmController { 

    public: 

        /**
        * @brief Constructor for CpmController Class
        * @param[in] M_i Inertial impedence parameter 
        * @param[in] B_i Dampening impedence parameter 
        * @param[in] K_i Spring impedence parameter 
        */
        CpmController(double M_i,double B_i,double K_i);


        /**
        * @brief Function to update torque values from RomBot object and a waypoint  
        * @param[in] robot RomBot object instance that contains robot parameter information 
        * @param[in] mjtWaypoint Array of doubles read from Serial that contains desired kinematic terms 
        */
        void updateTorques(RomBot& robot, double mjtWaypoint[]); 

        
        /**
        * @brief Getter for revolute joint torque 
        * @return desired torque at revolute joint taken directly from tau_[0]
        */
        double getThetaTorque(); 

        /**
        * @brief Getter for revolute joint torque 
        * @return desired torque at prismatic joint taken directly from tau_[1]
        */
        double getRTorque(); 
        
        /// Inertial Impedence Term 
        double M_i; 

        /// Damping Impedence Term
        double B_i;

        /// Spring-stiffness Impedence Term 
        double K_i; 

    private:
        
        /**
        * @brief Function to compute torque values for position-based impedence controller from RomBot object,a waypoint, and impedence params called inside of updateTorques  
        * @param[in] robot RomBot object instance that contains robot parameter information 
        * @param[in] qDesired Array of doubles for desired joint pos unpacked from mjtWaypoint 
        * @param[in] qdotDesired Array of doubles for desired joint vel unpacked from mjtWaypoint
        * @param[in] qddotDesired Array of doubles for desired joint acc unpacked from mjtWaypoint   
        */
        void computeTorques(RomBot& robot, double qDesired[],double qdotDesired[], double qddotDesired[]); 

        /// Gravity Constant  
        double g = 9.81;

        /// Waypoint array with desired q,qdot,qddot for both joints (size = 6) at discrete time step
        double mjtWayPoint[6]; 

        /// Position error array that contains e_x,e_y,e_theta 
        double errorEE_[3];

        /// Velocity error array that contains e_xdot,e_ydot,e_thetadot  
        double errorDotEE_[3];   

        /// Output torques 
        double tau_[2] = {0,0}; 

};
#endif // CPM_CONTROLLER_H