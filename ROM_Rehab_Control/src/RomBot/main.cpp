#include<iostream>
#include "RomBot.hpp"
#include "CpmController.hpp" 
int main() { 

    // Test Waypoint 
    double wayPoints[3][6] = {{0,0.18,0,0,0,0},{-0.004, 0.1798, -0.5343*(pow(10,-4)), -0.2337*(pow(10,-4)), -0.0106, -0.0046}, {-0.0026, 0.1788, -0.2104*(pow(10,-3)), -0.0921*(pow(10,-3)), -0.0207, -0.0091}}; 
    double wayPoint[6] = {0,0.18,0,0,0,0};


    // RomBot stuff  
    double rl = 0.2;
    double mass[3] = {1.0, 1.0, 0.5};
    double inertia[2] =  {1.5, 1.3}; 

    // Controller stuff
    double M_i = 0.5; 
    double B_i = 3; 
    double K_i = 2; 
    char type[20] = "Simple";

    // Create robot and controller obj
    RomBot robot(rl,mass,inertia); 
    CpmController cpm(M_i,B_i,K_i,type); 

    // Local variables to store current sensor measurements 
    double thetaSensed = -0.001; 
    double rSensed = 0.18003; 
    double thetadotSensed = 0; 
    double rdotSensed = 0; 
    double F_ext = 0; 

    robot.updateRomBot(thetaSensed,rSensed,thetadotSensed,rdotSensed,F_ext); 
    cpm.updateTorques(robot, wayPoint); 
    
    std::cout << "q: " << robot.getTheta() << "," << robot.getR() << "\n"; 
    std::cout << "qdot: " << robot.getTheta() << "," << robot.getRDot() << "\n"; 
    std::cout << "tau: " << cpm.getThetaTorque() << "," << cpm.getRTorque() << "\n"; 

    std::cin.get(); 
}