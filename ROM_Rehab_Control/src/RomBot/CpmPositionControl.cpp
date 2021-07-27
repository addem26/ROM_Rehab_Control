#include "CpmPositionControl.hpp" 

CpmPositionControl::CpmPositionControl(double M_i[],double B_i[],double K_i[],double sampleTime) { 
    // Populate impedence parameter arrays 
    for(int i = 0; i < 2; i++) { 
       this->M_i[i] = M_i[i];
       this->B_i[i] = B_i[i];
       this->K_i[i] = K_i[i];  
    }
    ts_ = sampleTime;
} 

void CpmPositionControl::updateImpedencePosition(RomBot& robot, double mjtWaypoint[]) { 
    
    // Unpack q and qDesired and l 
    double theta = robot.getTheta(); 
    double r = robot.getR(); 

    double thetaDesired = mjtWaypoint[0]; 
    double rDesired = mjtWaypoint[1]; 
    
    double l = robot.getRevoluteLength(); 

    // Calculate torque and force in correct coordinate system from sensor (reads only in y)

    double tau_z = robot.getExternalForce() * (l + r) * cos(theta); // For revolute Joint 
    double f_r = robot.getExternalForce() * sin(theta);  // For prismatic joint 

    // Difference equations to find current delta x (might have to map force from f_y to tau_z and f_r)
    double numTheta = (tau_z * pow(ts_,2)) + B_i[0] * (ts_ * errorTheta_[0]) + M_i[0] *(2*errorTheta_[0] - errorTheta_[1]); 
    double denTheta = M_i[0] + B_i[0]*ts_ + K_i[0]*pow(ts_,2); 
   
    double numR = (f_r * pow(ts_,2)) + B_i[1] * (ts_ * errorR_[0]) + M_i[1] *(2*errorR_[0] - errorR_[1]); 
    double denR = M_i[1] + B_i[1]*ts_ + K_i[1]*pow(ts_,2); 

    double deltaTheta =  numTheta/denTheta; 
    double deltaR = numR/denR; 

    // Update error arrays 
    errorTheta_[1] = errorTheta_[0]; 
    errorTheta_[0] = deltaTheta;  
    
    errorR_[1] = errorR_[0]; 
    errorR_[0] = deltaR;  

    // Update xc array 
    xc_[0] = thetaDesired + deltaTheta;
    xc_[1] = rDesired + deltaR;  

}

double CpmPositionControl::getThetaPos() {
    return xc_[0]; 
}   

double CpmPositionControl::getRPos() {
    return xc_[1]; 
}   

double * CpmPositionControl::getThetaError() { 
    return errorTheta_; 
}

double * CpmPositionControl::getRError() { 
    return errorR_; 
}