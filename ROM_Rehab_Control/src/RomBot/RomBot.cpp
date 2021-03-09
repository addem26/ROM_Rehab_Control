#include "RomBot.h" 
RomBot::RomBot(double ll,double hl, double m[]) : linkLength(ll), handleLength(hl) {
    //Fill mass array 
    for (int i  = 0; i < 2; i++) { 
        mass[i]  = m[i]; 
    } 
}

double RomBot::getLinkLength() { 
    return linkLength; 
}

double RomBot::gethandleLength() { 
    return handleLength; 
}

double * RomBot::getMasses() { 
    return mass; 
}

double RomBot::getPos() { 
    return pos; 
}

double RomBot::getVel() { 
    return vel; 
}

void RomBot::update(double new_pos, double new_vel) { 
    pos = new_pos; 
    vel = new_vel; 
}