#ifndef ROM_BOT_H
#define ROM_BOT_H
#include <Arduino.h> 
class RomBot { 
    private:
        //Robot Parameters 
        double linkLength;
        double handleLength;
        double mass[2];  
        
        //Kinematic Terms 
        double pos; 
        double vel;

    public:
        RomBot(double ll, double hl, double m[]); 
        
        //Getters b/c we do not want to change these values but we will 
        //use these for kinematics/dynamics/controller 
        double getLinkLength(); 
        double getHandleLength(); 
        double * getMasses(); 
        double getPos(); 
        double getVel(); 
        //Update From Sensor Values
        void update(double new_pos, double new_vel); 
}






#endif // RomBot 