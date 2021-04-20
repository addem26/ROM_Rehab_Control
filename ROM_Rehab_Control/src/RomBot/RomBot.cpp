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

void RomBot::differentialKinematics() { 
    xdot_[0] = qdot_[1] * cos(q_[0]); // xdot-component
    xdot_[1] = qdot_[1] * sin(q_[0]); // ydot-component
    xdot_[2] = qdot_[0]; // thetadot-component

}



//Complete this function  
void RomBot::forwardDyanmics(double tau[2]) { 
    qddot_[0] = 0; 
    qddot_[1] = 0; 
    
    xddot_[0] = qddot_[1] * cos(q_[0]) - qddot_[0]*qddot_[1] * sin(q_[0]); 
    xddot_[1] = qddot_[1] * sin(q_[0]) + qddot_[0]*qddot_[1] * cos(q_[0]); 
}

void RomBot::updateRomBot(double thetaSensed, double rSensed,double thetadotSensed, double rdotSensed, double extFSensed) { 
    
    // Update current angle, prismatic length, 
    q_[0] = thetaSensed; 
    q_[1] = rSensed;

    // Update change in angle,  prismatic length over time 
    qdot_[0] = thetadotSensed; 
    qdot_[1] = rdotSensed; 

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

double RomBot::getR() { 
    return q_[1]; 
}

double RomBot::getExternalForce() { 
    return externalForce_; 
}

double RomBot::getThetaDot() {
    return qdot_[0]; 
}

double RomBot::getRDot() { 
    return qdot_[1]; 
}


double *RomBot::getPosEE() { 
    return x_; 
}

double *RomBot::getVelEE() { 
    return xdot_; 
}

double *RomBot::getAccEE() { 
    return xddot_; 
} 

