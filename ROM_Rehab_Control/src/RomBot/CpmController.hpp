#ifndef CPM_CONTROLLER_H
#define CPM_CONTROLLER_H 
//#include <Arduino.h> 
#include <math.h> 
#include <cstring> 
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
        CpmController(double M_i,double B_i,double K_i,char *controller);


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
        
        /**
        * @brief Getter for Position Error  
        * @return Array of position error terms at the EE
        */
        double *getPosError(); 

        /**
        * @brief Getter for Velocity Error  
        * @return Array of velocity error terms at the EE
        */
        double *getVelError();

        /**
        * @brief Getter for Acceleration Error  
        * @return Array of acceleration error terms at the EE
        */
        double *getAccError();  
        
        /// Inertial Impedence Term 
        double M_i; 

        /// Damping Impedence Term
        double B_i;

        /// Spring-stiffness Impedence Term 
        double K_i; 

    private:
        
        /**
        * @brief Forward kinematics that takes the updated q_desired and maps to x_desired
        */
        void forwardKinematics(double l); 

                
        /**
        * @brief Differential Kinematics that take the updated qdot_desired and maps to xdot_deisred 
        */
        void differentialKinematics(); 


        /**
        * @brief Function to compute torque values based off of an impedence control scheme that the user decides. Called inside of updateTorques.   
        * @param[in] robot RomBot object instance that contains robot parameter information 
        * @param[in] qDesired Array of doubles for desired joint pos unpacked from mjtWaypoint 
        * @param[in] qdotDesired Array of doubles for desired joint vel unpacked from mjtWaypoint
        * @param[in] qddotDesired Array of doubles for desired joint acc unpacked from mjtWaypoint   
        */
        void computeTorques(RomBot& robot, double qDesired[],double qdotDesired[], double qddotDesired[]); 


        /**
        * @brief Function to calculate torque based off of impedence regulation at each joint (i.e. position and velocity error scaled by K_i and B_i for each joint)   
        * @param[in] robot RomBot object to get kinematic values read from sensors
        * @param[in] qDesired Array of doubles for desired joint pos unpacked from mjtWaypoint 
        * @param[in] qdotDesired Array of doubles for desired joint vel unpacked from mjtWaypoint
        */
        void simpleImpedence(RomBot& robot, double qDesired[],double qdotDesired[]); 


        /**
        * @brief Function to compute torque values based off of a position-based impedence control law that computes a cmd contact force as impedence at the EE (based off of kinematic error terms)
        *  and corrects for it throught the transpose of the jacobian 
        * @param[in] robot RomBot object instance that contains robot parameter information 
        * @param[in] qDesired Array of doubles for desired joint pos unpacked from mjtWaypoint 
        * @param[in] qdotDesired Array of doubles for desired joint vel unpacked from mjtWaypoint
        * @param[in] qddotDesired Array of doubles for desired joint acc unpacked from mjtWaypoint   
        */
        void positionBasedImpedence(RomBot& robot, double qDesired[],double qdotDesired[], double qddotDesired[]); 

        
        /**
        * @brief Function to compute torque values based off of a model-based impedence control law that computes desired joint torques with feedforward/feedback terms. This method requires in depth knowledge about 
        * the plant and assumes that we will be computing M(q) and N(q) online  
        * @param[in] robot RomBot object instance that contains robot parameter information 
        * @param[in] qDesired Array of doubles for desired joint pos unpacked from mjtWaypoint 
        * @param[in] qdotDesired Array of doubles for desired joint vel unpacked from mjtWaypoint
        * @param[in] qddotDesired Array of doubles for desired joint acc unpacked from mjtWaypoint   
        */
        void modelBasedImpedence(RomBot& robot, double qDesired[],double qdotDesired[], double qddotDesired[]); 


        /// Gravity Constant  
        double g = 9.81;

        /// Waypoint array with desired q,qdot,qddot for both joints (size = 6) at discrete time step
        double mjtWaypoint_[6]; 

        /// 
        double xDesired_[3];

        double xdotDesired_[3]; 

        double xddotDesired_[3];  

        /// Position error array that contains e_x,e_y,e_theta 
        double errorEE_[3];

        /// Velocity error array that contains e_xdot,e_ydot,e_thetadot  
        double errordotEE_[3];   

        /// Velocity error array that contains e_xddot,e_yddot,e_thetaddot  
        double errorddotEE_[3]; 

        /// Output torques 
        double tau_[2] = {0,0}; 

        /// Controller type 
        char *controller_; 

};
#endif // CPM_CONTROLLER_H