#include "RomBot.hpp" 
RomBot::RomBot(double revoluteLength, double mass[], double inertia[])  {
    //Fill mass array and inertia 
    for (int i  = 0; i < numBodies_; i++) { 
        mass_[i]  = mass[i]; 
    } 
    for (int i  = 0; i < numBodies_ - 1; i++) {
        inertia_[i] = inertia[i]; 
    }
    revoluteLength_ = revoluteLength; 
}

void RomBot::forwardKinematics() { 
    x_[0] = cos(q_[0]) * (q_[1] + revoluteLength_); // x-component
    x_[1] = sin(q_[0]) * (q_[1] + revoluteLength_); // y-component
    x_[2] = q_[0]; // theta-component
}

double *RomBot::fK(double q[]) { 

    double *x = new double[3]; 

    x[0] = cos(q[0]) * (q[1] + revoluteLength_); // x-component
    x[1] = sin(q[0]) * (q[1] + revoluteLength_); // y-component
    x[2] = q[0]; // theta-component

    return x; 
}

void RomBot::differentialKinematics() { 
    xdot_[0] = qdot_[1] * cos(q_[0]); // xdot-component
    xdot_[1] = qdot_[1] * sin(q_[0]); // ydot-component
    xdot_[2] = qdot_[0]; // thetadot-component

}

double *RomBot::dK(double q[], double qdot[]) { 
    
    double *xdot = new double[3]; 

    xdot[0] = qdot[1] * cos(q[0]); // xdot-component
    xdot[1] = qdot[1] * sin(q[0]); // ydot-component
    xdot[2] = qdot[0]; // thetadot-component

    return xdot; 
}

void RomBot::updateRomBot(double thetaSensed, double dSensed, double extFSensed, double pwm1, double pwm2) { 
    
    // Update current angle, prismatic length, 
    q_[0] = thetaSensed; 
    q_[1] = dSensed;

    // Update change in angle,  prismatic length over time 
    qdot_[0] = pwm1; 
    qdot_[1] = pwm2; 

    // Update extForce at the EE
    externalForce_ = extFSensed;  

    // Update x
    forwardKinematics(); 

    // Update xdot
    differentialKinematics(); 

    
}

const double RomBot::getRevoluteLength() { 
    return revoluteLength_; 
}

const double *RomBot::getMass() const { 
    return mass_; 
}

const double *RomBot::getInertia() const { 
    return inertia_; 
}

double RomBot::getTheta() { 
    return q_[0]; 
}

double RomBot::getD() { 
    return q_[1]; 
}

double RomBot::getExternalForce() { 
    return externalForce_; 
}

double RomBot::getThetaDot() {
    return qdot_[0]; 
}

double RomBot::getDDot() { 
    return qdot_[1]; 
}


double *RomBot::getPosEE() { 
    return x_; 
}

double *RomBot::getVelEE() { 
    return xdot_; 
}

