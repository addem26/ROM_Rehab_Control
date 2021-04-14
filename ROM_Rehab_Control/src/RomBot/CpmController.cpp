#include "CpmController.hpp"

CpmController::CpmController(double M_i, double B_i, double K_i) { 
    this->M_i = M_i; 
    this->B_i = B_i; 
    this->K_i = K_i; 
}

void CpmController::updateTorques(RomBot& robot, double mjtWaypoint[]) { 
    // Init Desired states from mjtWaypoint 
    double qDesired[2]; 
    double qdotDesired[2];  
    double qddotDesired[2]; 

    // Desired joint pos
    qDesired[0] = mjtWayPoint[0];
    qDesired[1] = mjtWayPoint[1]; 
    
    // Desired joint vel
    qdotDesired[0] = mjtWayPoint[2]; 
    qdotDesired[1] = mjtWayPoint[3];
    
    // Desired joint acc
    qddotDesired[0] = mjtWayPoint[4]; 
    qddotDesired[1] = mjtWayPoint[5]; 


    // Get Current states of the EE based off RomBot obj (fk and dk are run inside RomBot class here)
    double *x = robot.getPosEE(); 
    double *xdot = robot.getVelEE(); 

    // Get desired positions and velocites at EE (delete these after computation due to dynamic allocation)
    double *xDesired = robot.fK(qDesired);
    double *xdotDesired = robot.dK(qDesired,qdotDesired); 

    // Compute error terms and populate arrays
    for(int i = 0; i < 3; ++i) { 
        errorEE_[i] = x[i] - xDesired[i]; 
    }

    for(int i = 0; i < 3; ++i) {
        errorDotEE_[i] = xdot[i] - xdotDesired[i];  
    }

    // Delete dynamically allocated variables 
    delete[] xDesired; 
    delete[] xdotDesired; 

    computeTorques(robot,qDesired,qdotDesired,qddotDesired); 

}

void CpmController::computeTorques(RomBot& robot, double qDesired[],double qdotDesired[], double qddotDesired[]) { 
    // Get physical parameters from robot 
    const double l = robot.getRevoluteLength(); 
    const double *mass = robot.getMass(); 
    const double *i  = robot.getInertia(); 

    // Unpack joint kinematic terms in local variables 
    double theta = qDesired[0]; 
    double r = qDesired[1]; 

    double thetadot = qdotDesired[0]; 
    double rdot = qdotDesired[1]; 

    double thetaddot = qddotDesired[0]; 
    double rddot = qddotDesired[1]; 

    //Get force in and store in local variable and compute torque
    double force = robot.getExternalForce(); 
    double tauZ = force * (r + l)*cos(theta); 


    // Unpack masses of revolute link, prismatic, and handle and store in local variables 
    double mL = mass[0]; 
    double mR = mass[1]; 
    double mH = mass[2];  
    


    //Store error math and gravity matrix math for revolute joint in local variable
    double errorTermsTheta = qddotDesired[0] - (tauZ + B_i*errorDotEE_[0] + (K_i/M_i)*errorEE_[0])*(2*i[0] + 0.25*pow(l,2)*mL + mR*pow((0.5*r + l),2));
    double gravityTermsTheta = mH*g*sin(theta)*(r + l) + 0.5*mH*g*l*sin(theta) + 0.5*mR*g*sin(theta)*(r + 2l) - tauZ; 
    
    //Store error math and gravity matrix math for prismatic joint in local variable and compute acc at the EE 
    double xddotDesired = rddot * cos(theta) - rdot * sin(theta); 
    double yddotDesired = rddot * sin(theta) + rdot * cos(theta); 
     

    double errorTermsRx = 0.25*mR*cos(theta)*(xddotDesired - (B_i*errorDotEE_[0] + K_i*errorEE_[0])/M_i)); 
    double errorTermsRy = 0.25*mR*sin(theta)*(yddotDesired - (force + B_i*errorDotEE_[1] + K_i*errorEE_[1])/M_i); // f_{y]sin(theta)?  
    double gravityTermsR = mH*g*sin(theta)*(r + l)  + 0.5*mR*g*sin(r + 2*l); 

    
    tau_[0] = errorTermsTheta + gravityTermsTheta; //compute tau_theta 
    tau_[1] = errorTermsRx + errorTermsRy + gravityTermsR; //compute tau_r  
}

double CpmController::getThetaTorque() { 
    return tau_[0]; 
}

double CpmController::getRTorque() { 
    return tau_[1]; 
}