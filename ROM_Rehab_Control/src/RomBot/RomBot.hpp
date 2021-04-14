#ifndef ROM_BOT_H
#define ROM_BOT_H
//#include <Arduino.h> 
#include <math.h>
//! A class for the parameters of a RP rehabilitation robot
/*!
   This class contains all the necessary parameters for running control algorithms for an at home
   rehabilitation robot. The primary parameters include the states of all joints and the End-Effector (EE) as well as numerous
   dynamic paramters. 
*/
class RomBot { 

    public:

        /**
        * @brief Constructor for RomBot Class
        * @param[in] revoluteLength Length of revolute link
        * @param[in] mass Array of doubles that contain masses for the two links and handle (size = numBodies_) 
        * @param[in] inertia Array of doubles that contain the inertial terms for the two links and handle (size = numBodies_) 
        */
        RomBot(double revoluteLength, double mass[], double inertia[]); 
        


        /**
        * @brief Function to update parameter values from sensor mesurements
        * @param[in] thetaSensed Position of revolute joint read from encoder
        * @param[in] dSensed Postion of prismatic joint read from encoder 
        * @param[in] extFSensed Force at the EE read from a force sensor  
        * @param[in] pwm1 PWM signal for revolute joint to covert to qdot_[0]
        * @param[in] pwm2 PWM signal for prismatic joint to covert to qdot_[1]
        */
        void updateRomBot(double thetaSensed, double dSensed, double extFSensed,double pwm1, double pwm2); 

        /**
        * @brief A public Forward kinematics function 
        * @param[in] q 2D array containing values for theta and r
        * @return A dynamically allocated array that contains the position mapping from joint space to task space 
        */
        double *fK(double q[]);

        
        /**
        * @brief A public Differential kinematics function 
        * @param[in] q 2D array containing values for theta and r
        * @param[in] qdot 2D array containing values for thetaDot and rDot
        * @return A dynamically allocated array that contains the velocity mapping from joint space to task space for velocity
        */
        double *dK(double q[], double qdot[]);  


        /**
        * @brief Getter for length of revolute linkage  
        * @return Length as const double 
        */
        const double getRevoluteLength(); 


        /**
        * @brief Getter for mass array of size numBodies_ 
        * @return Const double pointer to first element in array 
        */
        const double *getMass() const; 

        /**
        * @brief Getter for inertia array of size numBodies_  
        * @return Const double pointer to first element in array 
        */
        const double *getInertia() const; 

        /**
        * @brief Getter for theta position
        * @return theta taken directly from q_[0]
        */
        double getTheta(); 

        /**
        * @brief Getter for d position 
        * @return theta taken directly from q_[1]
        */
        double getD(); 

        
        /**
        * @brief Getter for externalForce position 
        * @return Current externalForce_ read from force sensor 
        */
        double getExternalForce(); 

        /**
        * @brief Getter for theta velocity of 
        * @return thetaDot taken directly from qdot_[0]
        */
        double getThetaDot(); 

        /**
        * @brief Getter for theta velocity  
        * @return thetaDot taken directly from qdot_[1]
        */
        double getDDot(); 

        /**
        * @brief Getter for postion of the EE (x)  
        * @return double pointer to first element in array
        */
        double *getPosEE(); 
        
        /**
        * @brief Getter for velocity of the EE (xdot)  
        * @return double pointer to first element in array
        */
        double *getVelEE(); 



    private:

        /**
        * @brief Forward kinematics that takes the updated q and maps to x
        */
        void forwardKinematics(); 
        
        /**
        * @brief Differential Kinematics that take the updated qdot and maps to xdot
        */
        void differentialKinematics(); 


        /// Number of Rigid bodies in Dynamic System 
        static const int numBodies_ = 3; 

        /// Number of Degrees-Of-Freedom 
        static const int DOF_ = 2;

        /// Length of revolute joint  
        double revoluteLength_;

        /// Length of handle
        double handleLength_;

        /// Mass array 
        double mass_[numBodies_];

        /// Inertia Array 
        double inertia_[numBodies_];   
        
        /// Position array [theta , d]         
        double q_[DOF_]; 

        /// Velocity array [thetadot, ddot]
        double qdot_[DOF_]; 

        /// Postion of EE [x,y,theta] (size = DOF on a 2D plane)
        double x_[3];

        /// Velocity of EE [xdot,ydot,thetadot] (size = DOF on a 2D plane)
        double xdot_[3]; 

        /// External Force at the EE
        double externalForce_ = 0; 
        
};
#endif // ROM_BOT_H 