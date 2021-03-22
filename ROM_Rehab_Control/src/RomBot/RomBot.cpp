#include "RomBot.h" 
RomBot::RomBot(double revoluteLength,double handleLength, double mass[], double inertia[])  {
    //Fill mass array and inertia 
    for (int i  = 0; i < numBodies_; i++) { 
        this->mass_[i]  = mass_[i]; 
    } 
    for (int i  = 0; i < numBodies_; i++) {
        this->inertia_[i] = inertia_[i]; 
    }
    this->revoluteLength_ = revoluteLength; 
    this->handleLength_ = handleLength; 
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

const double RomBot::getRevoluteLength() { 
    return revoluteLength_; 
}

const double RomBot::getHandleLength() { 
    return handleLength_; 
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

void RomBot::updateRomBot(double thetaSensed, double dSensed, double extFSensed, double pwm1, double pwm2) { 
    
    //Update current angle, prismatic length, and extForce at the EE
    q_[0] = thetaSensed; 
    q_[1] = dSensed;

    //????
    qdot_[0] = pwm1; 
    qdot_[1] = pwm2; 

    externalForce_ = extFSensed;  

    //Update x
    forwardKinematics(); 

    //Update xdot
    differentialKinematics(); 

    
}