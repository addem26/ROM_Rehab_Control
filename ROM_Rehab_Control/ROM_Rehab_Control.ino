#include "src/RomBot/RomBot.hpp"
/*
 * Test std::vector
 */
double mass[3] = {1,1,1}; 
double inertia[3] = {1,1,1}; 
double revoluteLength = 1.0; 
double handleLength = 0.5; 

//double mesurements1 = {
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600); 

  RomBot bot(handleLength,revoluteLength,mass,inertia); 
  //Serial.print(bot.getRevoluteLength()); 
 const double *ma = bot.getMass(); 

  for(int i = 0; i  < 3; i++) { 
    Serial.print(ma[i]);
    Serial.print("\n");  
  }
  const double *no = bot.getMass(); 
  Serial.print(no[0]*10);  
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second


}
