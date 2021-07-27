function [q,qd,qdd] = mjtTrajectoryGeneration(ROM,vel_avg,reps,P) 

%Init path and empty joint state vector 
jointStates = [];
scaling_value = roots(P);
x = scaling_value(1):-0.5:(scaling_value(1) - 15);
y = P(1)*x.^4 + P(2)*x.^3 + P(3)*x.^2 + P(4)*x + P(5);
figure(1)
plot(x,y)
%Loop through path points and compute IK 
for i = 1:size(x,2)
    
    [theta,d] = analyticIK(x(i),y(i),35); 
    
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

figure(2)
hold on
plot(T, theta)
plot(T, d)
hold off
%Run MJT Generator 
[t,Cj,PPj,VVj,AAj,POSj,VELj,ACCj,Pj,Vj,Aj] = MinimumJerkGenerator(T,Pos,v0,vf,a0,af);


%Store generated trajectory in workspace for Simulink
q = timeseries(Pj(1:2,:),t); 
qd = timeseries(Vj(1:2,:),t); 
qdd = timeseries(Aj(1:2,:),t); 

end