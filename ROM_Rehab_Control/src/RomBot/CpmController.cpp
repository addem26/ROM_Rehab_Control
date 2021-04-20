#include "CpmController.hpp"

CpmController::CpmController(double M_i, double B_i, double K_i,char *controller) { 
    this->M_i = M_i; 
    this->B_i = B_i; 
    this->K_i = K_i; 
    controller_ = controller; 
}



void CpmController::forwardKinematics(double l) { 
    xDesired_[0] = cos(mjtWaypoint_[0]) * (mjtWaypoint_[1] + l); // x-component
    xDesired_[1] = sin(mjtWaypoint_[0]) * (mjtWaypoint_[1] + l); // y-component
    xDesired_[2] = mjtWaypoint_[0]; // theta-component
}

void CpmController::differentialKinematics() { 
    xdotDesired_[0] = mjtWaypoint_[3] * cos(mjtWaypoint_[0]); // xdot-component
    xdotDesired_[1] = mjtWaypoint_[3] * sin(mjtWaypoint_[0]); // ydot-component
    xdotDesired_[2] = mjtWaypoint_[2]; // thetadot-component

}

void CpmController::updateTorques(RomBot& robot, double mjtWaypoint[]) { 
    // Init Desired states from mjtWaypoint 
    double qDesired[2]; 
    double qdotDesired[2];  
    double qddotDesired[2]; 

    for (int i = 0;  i < 6; ++i) { 
        mjtWaypoint_[i] = mjtWaypoint[i]; 
    }

    //Map desired joint space kinematics to desired task space kinematics for impedence control 
    forwardKinematics(robot.getRevoluteLength()); 
    differentialKinematics(); 
    //forwardDynamics(); 

    // Get Current states of the EE based off of RomBot obj (fk and dk are run inside RomBot class here)
    double *x = robot.getPosEE(); 
    double *xdot = robot.getVelEE(); 
    double *xddot = robot.getAccEE(); 

    
    // Compute error terms and populate arrays
    for(int i = 0; i < 3; ++i) { 
        errorEE_[i] = x[i] - xDesired_[i]; 
        errordotEE_[i] = xdot[i] - xdotDesired_[i];  
        errorddotEE_[i] = xddot[i] - xddotDesired_[i]; 
    }

    computeTorques(robot,qDesired,qdotDesired,qddotDesired); 

}

void CpmController::computeTorques(RomBot& robot, double qDesired[],double qdotDesired[], double qddotDesired[]) { 

    if (strcmp(controller_,"ModelBasedImp") == 0) { 
        modelBasedImpedence(robot,qDesired,qdotDesired,qddotDesired); 
    }
    else if(strcmp(controller_, "PositionBasedImp") == 0) {
        positionBasedImpedence(robot,qDesired,qdotDesired,qddotDesired); 
    }
    else
        simpleImpedence(robot,qDesired,qdotDesired); 
}


void CpmController::simpleImpedence(RomBot& robot, double qDesired[],double qdotDesired[]) { 
    
    double theta = robot.getTheta(); 
    double thetadot = robot.getThetaDot(); 
    double thetaDesired = mjtWaypoint_[0]; 
    double thetadotDesired = mjtWaypoint_[2]; 

    double r = robot.getR(); 
    double rdot = robot.getRDot(); 
    double rDesired = mjtWaypoint_[1]; 
    double rdotDesired = mjtWaypoint_[3]; 

    tau_[0] = K_i * (thetaDesired - theta) + B_i * (thetadotDesired  - thetadot);   
    tau_[1] = K_i * (rDesired - r) + B_i * (rdotDesired  - rdot);   

}

void CpmController::positionBasedImpedence(RomBot& robot, double qDesired[],double qdotDesired[], double qddotDesired[]) { 
    //Update observed acceleration from torque commands 
    robot.forwardDyanmics(tau_); 

}
// In Progress
void CpmController::modelBasedImpedence(RomBot& robot, double qDesired[],double qdotDesired[], double qddotDesired[]) { 
        // Get physical parameters from robot 
    const double l = robot.getRevoluteLength(); 
    const double *mass = robot.getMass(); 
    const double *i  = robot.getInertia(); 

    // Unpack joint kinematic terms into local variables 
    double theta = robot.getTheta();  
    double r = robot.getR(); 

    double thetadot = robot.getThetaDot(); 
    double rdot = robot.getRDot();  

    double thetaddotDesired = qddotDesired[0]; 
    double rddotDesired = qddotDesired[1]; 

    //Get force and store in local variable and compute torque about the z-axis 
    double force = robot.getExternalForce(); 
    double tauZ = force * (r + l)*cos(theta); 


    // Unpack masses of revolute link, prismatic link, and handle and store in local variables 
    double mL = mass[0]; 
    double mR = mass[1]; 
    double mH = mass[2];  
    


    //Store error math and gravity matrix math for revolute joint in local variables
    double errorTermsTheta = thetaddotDesired - (tauZ + B_i*errordotEE_[0] + (K_i/M_i)*errorEE_[0])*(2*i[0] + 0.25*pow(l,2)*mL + mR*pow((0.5*r + l),2));
    double gravityTermsTheta = mH*g*sin(theta)*(r + l) + 0.5*mH*g*l*sin(theta) + 0.5*mR*g*sin(theta)*(r + 2l) - tauZ; 
    

      
    // Compute acc at the EE
    double xddotDesired = 1 * cos(theta) - rdot * sin(theta); 
    double yddotDesired = 1 * sin(theta) + rdot * cos(theta); 
     
    //Store error math and gravity matrix math for prismatic joint in local variables 
    //NEED TO ADD DERIVATIVE OF JACOBIAN TIMES VELOCITYYYYY
    double errorTermsRx = 0.25*mR*cos(theta)*(xddotDesired - (B_i*errordotEE_[0] + K_i*errorEE_[0])/M_i); 
    double errorTermsRy = 0.25*mR*sin(theta)*(yddotDesired - (force + B_i*errordotEE_[1] + K_i*errorEE_[1])/M_i); // f_{y]sin(theta)?  
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

double *CpmController::getPosError() { 
    return errorEE_; 
}


double *CpmController::getVelError() { 
    return errordotEE_; 
}


double *CpmController::getAccError() { 
    return errorddotEE_; 
}
