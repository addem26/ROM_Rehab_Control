function traj = trajectory_generation(rom,vel,reps) 

rom_rad = rom * (pi/180); 
omega = 2*pi/(vel*1*reps);
time  = 0:0.01:reps;
traj = rom_rad * cos(time) - rom_rad; 
plot(time,traj); 

end 