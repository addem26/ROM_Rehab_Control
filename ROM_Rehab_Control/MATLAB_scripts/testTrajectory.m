% Define Trajectory and Input Parameters 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Define desired ROM input and Average Velocity 
%Going to implement curve type depending on length of 
%fingers and reps soon 
ROM = pi/2;
vel = 0.07;
reps = 0;
%Store generated trajectory in workspace for Simulink
[q_desired,qd_desired,qdd_desired] = mjtTrajectoryGeneration(ROM,vel,reps); 

theta_unformat = q_desired.data(1,1,:); 
theta = reshape(theta_unformat,1,size(theta_unformat,3)); 

r_unformat = q_desired.data(2,1,:); 
r = reshape(r_unformat,1,size(r_unformat,3)); 

theta_deg = theta.*(180/pi); 

rep_theta = [theta_deg flip(theta_deg)];
rep_r = [r flip(r)]; 
time = 0:0.01:q_desired.time(end)*2; 


y = [rep_theta;rep_r]; 
fileID = fopen('trajectory.txt','w'); 
fprintf(fileID,'Desired Trajectory [theta,r]\n\n'); 
fprintf(fileID,'%f %f\n',y); 
fclose(fileID); 
type trajectory.txt

figure(1)
plot(time,rep_theta(1:end - 1))
title("Desired Revolute Position vs Time")
xlabel("Time [s]")
ylabel("Position [deg]")


figure(2) 
plot(time,rep_r(1:end - 1))
title("Desired Prismatic Position vs Time")
xlabel("Time [s]")
ylabel("Position [mm]")