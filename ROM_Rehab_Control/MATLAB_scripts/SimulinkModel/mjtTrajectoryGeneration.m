function [q,qd,qdd] = mjtTrajectoryGeneration(ROM,vel_avg,reps) 
%Plot path 
z = 0.38:-0.015:0; 
x = ones(size(z,2)).*0.2; 
[X,Z] = meshgrid(x,z); 
y = -sqrt((0.38 - Z)/120); 
surf(X,Z,y);

%Init path and empty joint state vector 
jointStates = [];
x = 0.38:-0.015:0; 
y = -sqrt((0.38 - x)/120); 

%Loop through path points and compute IK 
for i = 1:size(x,2)
    
    [theta,d] = analyticIK(x(i),y(i),0.2); 
    
    %If theta is passed ROM then this is the end of the trajectory
    if theta > ROM || theta  < -ROM 
        break 
    end
    jointState = [theta,d];
    jointStates = [jointStates;jointState]; 
end 

%Calculate total time span 
time = abs(jointStates(1,1) - jointStates(end,1)) / vel_avg; 

%Init time, desired postions, and terminal conditions 
T = linspace(0,time,size(jointStates,1));
Pos = [jointStates zeros(size(jointStates,1),1)]; 
v0 = [0,0,0];
vf = [0,0,0];
a0 = [0,0,0];
af = [0,0,0];

%Run MJT Generator 
[t,Cj,PPj,VVj,AAj,POSj,VELj,ACCj,Pj,Vj,Aj] = MinimumJerkGenerator(T,Pos,v0,vf,a0,af);


%Store generated trajectory in workspace for Simulink
q = timeseries(Pj(1:2,:),t); 
qd = timeseries(Vj(1:2,:),t); 
qdd = timeseries(Aj(1:2,:),t); 

end