#include "src/RomBot/RomBot.hpp"
#include "src/RomBot/CpmPositionControl.hpp"
//Robot stuff
double revoluteLength = 1; 
double mass[3] = {1.0,1.0,0.5};  
double inertia[3] = {0,0}; 

//Controller stuff
double Md[2] = {1.0,1.0}; 
double Bd[2] = {2.5,2.5}; 
double Kd[2] = {30,30}; 
double ts = 0.001; 

//Setpoint 
double wayPoint[2] = {5,3}; 

RomBot robot(revoluteLength, mass, inertia); 
CpmPositionControl cpm(Md,Bd,Kd,ts); 


double thetaSensed = 3; 
double rSensed = 1; 
double F_ext = 30; 

double xc[2]; 
void setup() {
  // initialize digital pin LED_BUILTIN as an output.

  Serial.begin(9600); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Native USB only
  }
  Serial.println("Hello"); 

}

// the loop function runs over and over again forever
void loop() {
  
  robot.updateRomBot(thetaSensed,rSensed, F_ext); 
  cpm.updateImpedencePosition(robot, wayPoint); 

  xc[0] = cpm.getThetaPos(); 
  xc[1] = cpm.getRPos(); 
  Serial.println(xc[0]); 
  F_ext = 0; 
}
