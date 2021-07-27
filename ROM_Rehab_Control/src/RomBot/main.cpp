//#include<iostream>
#include "RomBot.hpp"
#include "CpmController.hpp" 
#include "CpmPositionControl.hpp"
//#include <fstream>
//#include <iostream> 
int main() { 

    // Test Waypoint 
    double wayPoint[2] = {15,2};

    double ts = 0.007; 
    // RomBot - Init length for revolute arm, masses for bodies, and inertia 
    double rl = 0.2;
    double mass[3] = {1.0, 1.0, 0.5};
    double inertia[2] =  {1.5, 1.3}; 

    // Controller - Init desired impedence paramters (might change this to array), and type of controller
    double M_i[2] = {1.0,1.0}; 
    double B_i[2] = {2.5,2.5}; 
    double K_i[2] = {25,25}; 
    

    // Create robot and controller obj
    RomBot robot(rl,mass,inertia); 
    CpmPositionControl cpm(M_i,B_i,K_i,ts); 

    double theta_pos[1000]; 
    double r_pos[1000]; 
    // Local variables to store current sensor measurements 
    double thetaSensed = 3; 
    double rSensed = 0.2; 
    double F_ext = 1000; 
    /*
    std::ofstream myfile;
    myfile.open("example.txt");
    myfile << "[theta , r]\n";
    for(int i = 0;  i <  1000; i++) { 
        /*
        if(i == 200){ 
            F_ext = 60; 
        }
        robot.updateRomBot(thetaSensed,rSensed,F_ext); 
        cpm.updateImpedencePosition(robot, wayPoint); 
        
        theta_pos[i] = cpm.getThetaPos(); 
        r_pos[i] = cpm.getRPos();

 
        myfile << cpm.getThetaPos() << " " << cpm.getRPos() << "\n";
        F_ext = 0; 
    }
    myfile.close();
    // Update robot obj and controller obj from sensor values and waypoint 

    
    std::cout << "q: " << robot.getTheta() << "," << robot.getR() << "\n"; 
    std::cout << "qdot: " << robot.getTheta() << "," << robot.getRDot() << "\n"; 
    std::cout << "tau: " << cpm.getThetaPos() << "," << cpm.getRPos() << "\n"; 

    for (auto pos : theta_pos)  {
        std::cout << "theta: " << pos << "\n"; 
    }
    for (auto pos : r_pos)  {
        std::cout << "r: " << pos << "\n"; 
    }

    std::cin.get(); 
    */
}