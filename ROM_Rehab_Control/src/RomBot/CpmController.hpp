#ifndef CPM_CONTROLLER_H
#define CPM_CONTROLLER_H 
#include <Arduino.h> 
#include <math.h> 
#include "RomBot.hpp" 

class CpmController { 

    public: 
        CpmControler(RomBot& robot, , double mjtWaypoint[]);
        
        /// Inertial Impedence Term 
        double M_i; 

        /// Damping Impedence Term
        double B_i;

        /// Spring-stiffness Impedence Term 
        double K_i; 


    private:
        
        /// Gravity Constant  
        double g = 9.81;

        double mjtWayPoint[6]; 

        double errorEE_[3];

        double errorDotEE_[3];  

        double tauZ_; 


};