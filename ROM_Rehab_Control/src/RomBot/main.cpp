#include<iostream>
#include "RomBot.hpp"

int main() { 
    double rl = 0.5;
    double mass[3] = {1.0, 1.0, 0.5};
    double inertia[2] =  {1.5, 1.3}; 

    RomBot bot(rl,mass,inertia); 
    const double *i_local = bot.getInertia(); 
    std::cout << "Hello world" << "\n"; 
    //std::cout << i_local[1] << "\n";

    std::cout << pow(5,2);   

    std::cin.get(); 
}