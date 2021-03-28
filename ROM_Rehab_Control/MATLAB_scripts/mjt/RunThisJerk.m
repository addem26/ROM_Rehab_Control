clear all;close all;clc;
% This is a tool that can be used to generate and plot the multi-segment minimum jerk trajectory that satisfies the following:
% 1) Passes through any number of waypoints at certain times.
% 2) Satisfies certain boundary conditions for velocity and acceleration (Initial and final velocity and acceleration).
% 
% 
% Features:
% 1) It is a straightforward package as everything is in one file and all the user needs is to enter very few inputs, then run the code.
% 2) It is general, which means that it works with  2D and 3D trajectories and can deal with any number of waypoints.
% 
% Inputs:
% 1) Positions of the waypoints (2D or 3D).
% 2) The desired time of each waypoint.
% 3) The desired initial and final velocity and acceleration.
vel = 0.05; 
x = 0:-0.15:-1; 
y = -x.^2 + 1; 
figure(1)
plot(y,x)
jointStates = [];

for i = 1:size(x,2) 
    [theta,d] = analyticIK(x(i),y(i),0.5); 
    jointState = [theta,d];
    jointStates = [jointStates;jointState]; 
end 


time = abs(jointStates(1,1) - jointStates(end,1)) / vel 
T = linspace(0,time,size(jointStates,1))
Pos = [jointStates zeros(size(jointStates,1),1)]; 
v0 = [0,0,0]
vf = [0,0,0] 
a0 = [0,0,0]
af = [0,0,0]


% You can use the following part as an example on how this code works and how to input.
% You just need to enter your desired values into the following vectors and matrices, then run the code ... IT IS THAT SIMPLE!
%T=[0 4 8 12]; %Time of waypoints ... they must be in order. 
%Pos=[0 0 0;
%     2 0 0;
%     1 sqrt(3) 0;
%     0 0 0]; % Positions of waypoints ... Each row represents (x y z) of a waypoint ... they must be in order.
             %If you have a 2D trajectory, then your matrix should have 2 columns instead of 3. 
             
% v0=[0,0,0]; % Initial velocity (the components are in 3D (x y z)) [MUST have the same length as the number of columns of Pos].
             % If you have a 2D trajectory, then your vector size should be 1*2 NOT 1*3. 
% vf=[0,0,0]; % Final velocity (the components are in 3D (x y z)) [MUST have the same length as the number of columns of Pos].
              % If you have a 2D trajectory, then your vector size should be 1*2 NOT 1*3. 
% a0=[0,0,0]; % Initial acceleration (the components are in 3D (x y z)) [MUST have the same length as the number of columns of Pos].
              % If you have a 2D trajectory, then your vector size should be 1*2 NOT 1*3. 
% af=[0,0,0]; % Final acceleration (the components are in 3D (x y z)) [MUST have the same length as the number of columns of Pos].
              % If you have a 2D trajectory, then your vector size should be 1*2 NOT 1*3. 
                      
% Outputs:
% 1) The simulation time vector (t) in case the user wants to perform further analysis.
% 2) The Coefficient Vector (C), which contains all the coefficients of the 5th degree polynomials of the position trajectory segments. It has a dimension (6n), where n is the number of trajectory segments. The first 6 elements are the coefficients of the first trajectory segment, the second 6 elements are for the second segment and so on so forth. for each segment, the first coefficient is associated with (t^5) and the so on until the 6th coefficient in the segment which is associated with (t^0).
% 3) The minimum jerk trajectory and the corresponding trajectory for the velocity and the acceleration (PP, VV, AA) each as a matrix of size (d,n), where d is the dimension of the problem (2D or 3D) and n is the number of trajectory segments. 
% 4) The minimum jerk trajectory and the corresponding trajectory for the velocity and the acceleration (POS, VEL, ACC) each as a piecewise function of time. This is a piecewise matrix of size (d,n), where d is the dimension of the problem (2D or 3D) and n is the number of trajectory segments. 
% 5) The minimum jerk trajectory and the corresponding trajectory for the velocity and the acceleration (P, V, A) each as a numerical matrix. The numerical values are obtained after substituting the trajectory time into the piecewise function. This is a matrix of size (tt,n), where tt is the dimension of the simulation time vector and n is the number of trajectory segments.  
% 6) Several plot such as: Position vs time (each direction on a separate subplot), Velocity vs time (each direction on a separate subplot), Acceleration vs time (each direction on a separate subplot), The path of motion (in 2D or 3D), The velocity path of motion (in 2D or 3D), The acceleration path of motion (in 2D or 3D). 
%              
[t,Cj,PPj,VVj,AAj,POSj,VELj,ACCj,Pj,Vj,Aj] = MinimumJerkGenerator(T,Pos,v0,vf,a0,af);
% For the definistions of each output, please go to the preivious paragraph under the title (Outputs).
