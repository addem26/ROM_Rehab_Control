function [t,Cj,PPj,VVj,AAj,POSj,VELj,ACCj,Pj,Vj,Aj]=MinimumJerkGenerator(T,Pos,v0,vf,a0,af)
%% Prelimenary Manupulations  
[rows,columns]=size(Pos);  %rows= number of waypoints ... columns= number of coordinates (1D, 2D or 3D) 
I=eye(columns);
n=rows-1;  %n= number of trajectories 

%% Waypoints' Matrix
R=zeros(6*columns*n,6*columns*n); % Start with zero matrix for R

% Positions of all waypoints
for i=1:1:n
     R(((1+(2*columns*(i-1))):(columns+(2*columns*(i-1)))),((1+(6*columns*(i-1))):(6*columns+(6*columns*(i-1)))))=[I*T(i)^5 I*T(i)^4 I*T(i)^3 I*T(i)^2 I*T(i)^1 I];
     R(((1+columns+(2*columns*(i-1))):(2*columns+(2*columns*(i-1)))),((1+(6*columns*(i-1))):(6*columns+(6*columns*(i-1)))))=[I*T(i+1)^5 I*T(i+1)^4 I*T(i+1)^3 I*T(i+1)^2 I*T(i+1)^1 I]; 
end     
% Velocity boundary conditions (inital and final waypoints)        
R(((1+(2*columns*n)):(columns+(2*columns*n))),(1:6*columns)) =[5*I*T(1)^4 4*I*T(1)^3 3*I*T(1)^2 2*I*T(1)^1 I zeros(columns,columns)];
R(((1+columns+(2*columns*n)):((2*columns)+(2*columns*n))),((1+(6*columns*(n-1))):(6*columns*n)))=[5*I*T(n+1)^4 4*I*T(n+1)^3 3*I*T(n+1)^2 2*I*T(n+1)^1 I zeros(columns,columns)];
% Equal Accelaration boundary conditions (initial and final waypoints)
R(((1+(2*columns)+(2*columns*n)):((3*columns)+(2*columns*n))),(1:6*columns)) =[20*I*T(1)^3 12*I*T(1)^2 6*I*T(1)^1 2*I zeros(columns,columns) zeros(columns,columns)];
R(((1+(3*columns)+(2*columns*n)):((4*columns)+(2*columns*n))),((1+(6*columns*(n-1))):(6*columns*n)))=[20*I*T(n+1)^3 12*I*T(n+1)^2 6*I*T(n+1)^1 2*I zeros(columns,columns) zeros(columns,columns)];
%Equal velocity, accelaration , jerk, and snap at intermideate waypoints
for i=1:n-1
R(((1+(columns*(i-1))+(4*columns)+(2*columns*n)):(columns+(columns*(i-1))+(4*columns)+(2*columns*n))),(1+(6*columns*(i-1)):6*(i+1)*columns)) =[5*I*T(i+1)^4 4*I*T(i+1)^3 3*I*T(i+1)^2 2*I*T(i+1)^1 I zeros(columns,columns) -5*I*T(i+1)^4 -4*I*T(i+1)^3 -3*I*T(i+1)^2 -2*I*T(i+1)^1 -I zeros(columns,columns)]; % Equal velocity at intermediate waypoints
R(((1+(columns*(i-1))+(columns*(n-1))+(4*columns)+(2*columns*n)):(columns+(columns*(i-1))+(columns*(n-1))+(4*columns)+(2*columns*n))),(1+(6*columns*(i-1)):6*(i+1)*columns)) =[20*I*T(i+1)^3 12*I*T(i+1)^2 6*I*T(i+1)^1 2*I zeros(columns,columns) zeros(columns,columns) -20*I*T(i+1)^3 -12*I*T(i+1)^2 -6*I*T(i+1)^1 -2*I zeros(columns,columns) zeros(columns,columns)]; % Equal acceleration at intermediate waypoints
R(((1+(columns*(i-1))+(2*columns*(n-1))+(4*columns)+(2*columns*n)):(columns+(columns*(i-1))+(2*columns*(n-1))+(4*columns)+(2*columns*n))),(1+(6*columns*(i-1)):6*(i+1)*columns)) =[60*I*T(i+1)^2 24*I*T(i+1)^1 6*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) -60*I*T(i+1)^2 -24*I*T(i+1)^1 -6*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns)]; % Equal jerk at intermediate waypoints
R(((1+(columns*(i-1))+(3*columns*(n-1))+(4*columns)+(2*columns*n)):(columns+(columns*(i-1))+(3*columns*(n-1))+(4*columns)+(2*columns*n))),(1+(6*columns*(i-1)):6*(i+1)*columns)) =[120*I*T(i+1)^1 24*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) -120*I*T(i+1)^1 -24*I zeros(columns,columns) zeros(columns,columns) zeros(columns,columns) zeros(columns,columns)]; % Equal snap at intermediate waypoints
end

%% Boundary Conditions Matrix
BC=zeros(18*n,1);
BC(1:columns,1:1)=(Pos(1,:)).'; % Position of the first waypoint
PosInter=[];
for i=2:1:n
    posinter=[Pos(i,:);Pos(i,:)];
    PosInter=[PosInter;posinter];
end
for i=1:1:2*(n-1)
    BC((1+columns+((i-1)*columns)):(columns+columns+((i-1)*columns)),1)=(PosInter(i,:)).';
end
BC(((1+columns+(2*(n-1)*columns)):(columns+columns+(2*(n-1)*columns))),1)=(Pos(end,:)).'; % Position of the final waypoint
BC(((1+(2*columns)+(2*(n-1)*columns)):(columns+(2*columns)+(2*(n-1)*columns))),1)=(v0).'; %initial velocity
BC(((1+(3*columns)+(2*(n-1)*columns)):(columns+(3*columns)+(2*(n-1)*columns))),1)=(vf).'; %final velocity
BC(((1+(4*columns)+(2*(n-1)*columns)):(columns+(4*columns)+(2*(n-1)*columns))),1)=(a0).'; %initial acceleration
BC(((1+(5*columns)+(2*(n-1)*columns)):(columns+(5*columns)+(2*(n-1)*columns))),1)=(af).'; %final acceleration

%% Coefficient  Vector
size(BC)
size(R)
Cj=R\BC;

%% Trajectory Ploynomial
syms t

PPj=sym(zeros(columns,n));
for i=1:1:n
  PPj(1:columns,i)=(Cj(((1+(6*(i-1)*columns)):(columns+(6*(i-1)*columns))),1)*t^5)+(Cj(((1+columns+(6*(i-1)*columns)):(2*columns+(6*(i-1)*columns))),1)*t^4)+(Cj(((1+2*columns+(6*(i-1)*columns)):(3*columns+(6*(i-1)*columns))),1)*t^3)+(Cj(((1+3*columns+(6*(i-1)*columns)):(4*columns+(6*(i-1)*columns))),1)*t^2)+(Cj(((1+4*columns+(6*(i-1)*columns)):(5*columns+(6*(i-1)*columns))),1)*t)+(Cj(((1+5*columns+(6*(i-1)*columns)):(6*columns+(6*(i-1)*columns))),1));
end

PPj=vpa(PPj);  % Minimal Jerk Trajectory ... Position ... Each COLUMN represents one of the trajectories between two points ... Number of columns = n
VVj=vpa(diff(PPj,t)); % Minimal Jerk Trajectory ... Velocity ... Each COLUMN represents one of the trajectories between two points ... Number of columns = n
AAj=vpa(diff(VVj,t)); % Minimal Jerk Trajectory ... Acceleration ... Each COLUMN represents one of the trajectories between two points ... Number of columns = n

POSj=piecewise(T(1)<=t<T(2),PPj(:,1));
if n>1
for i=2:1:n
POSj=piecewise(T(i)<=t<T(i+1),PPj(:,i),POSj);   %Piecewise POSITION trajectory (TIME function) ... REMOVE the semicolon if you want to display
end
end

VELj=piecewise(T(1)<=t<T(2),VVj(:,1));
if n>1
for i=2:1:n
VELj=piecewise(T(i)<=t<T(i+1),VVj(:,i),VELj);  %Piecewise VELOCITY trajectory (TIME function) ... REMOVE the semicolon if you want to display
end
end

ACCj=piecewise(T(1)<=t<T(2),AAj(:,1));
if n>1
for i=2:1:n
ACCj=piecewise(T(i)<=t<T(i+1),AAj(:,i),ACCj); %Piecewise ACCELERATION trajectory (TIME function) ... REMOVE the semicolon if you want to display 
end
end
 
t=T(1):1e-2:T(n+1);
Pj=double(subs(POSj)); %POSITION trajectory (NUMERICAL VALUES AFTER SUB TIME) ... REMOVE the semicolon if you want to display 
Vj=double(subs(VELj)); %VELOCITY trajectory (NUMERICAL VALUES AFTER SUB TIME) ... REMOVE the semicolon if you want to display
Aj=double(subs(ACCj)); %ACCELERATION trajectory (NUMERICAL VALUES AFTER SUB TIME) ... REMOVE the semicolon if you want to display
%% Ploting
%{
figure('WindowState', 'maximized')
for i=1:1:columns
subplot(columns,1,i)
txt=['Position in D_',num2str(i),' (m)' ];
plot(t,Pj(i,:),'DisplayName',txt,'LineWidth',3)
grid on
legend show;
ylabel(txt);
xlabel('Time (sec)')
title(txt);
set(gca,'FontSize',15);
end

figure('WindowState', 'maximized')
for i=1:1:columns
subplot(columns,1,i)
txt=['Velocity in D_',num2str(i),' (m/sec)' ];
plot(t,Vj(i,:),'DisplayName',txt,'LineWidth',3)
grid on
legend show;
ylabel(txt);
xlabel('Time (sec)')
title(txt);
set(gca,'FontSize',15);
end

figure('WindowState', 'maximized')
for i=1:1:columns
subplot(columns,1,i)
txt=['Acceleration in D_',num2str(i),' (m/sec^2)' ];
plot(t,Aj(i,:),'DisplayName',txt,'LineWidth',3)
grid on
legend show;
ylabel(txt);
xlabel('Time (sec)')
title(txt);
set(gca,'FontSize',15);
end

figure('WindowState', 'maximized')
if columns==2
  plot(Pj(1,:),Pj(2,:),'LineWidth',3) 
  grid on
  xlabel('D_1')
  ylabel('D_2')
  title('Path')
  set(gca,'FontSize',15);
elseif columns==3
  plot3(Pj(1,:),Pj(2,:),Pj(3,:),'LineWidth',3) 
  grid on
  xlabel('D_1')
  ylabel('D_2')
  zlabel('D_3')
  title('Path')
  set(gca,'FontSize',15);
end

figure('WindowState', 'maximized')
if columns==2
  plot(Vj(1,:),Vj(2,:),'LineWidth',3) 
  grid on
  xlabel('D_1')
  ylabel('D_2')
  title('Velocity Path')
  set(gca,'FontSize',15);
elseif columns==3
  plot3(Vj(1,:),Vj(2,:),Vj(3,:),'LineWidth',3) 
  grid on
  xlabel('D_1')
  ylabel('D_2')
  zlabel('D_3')
  title('Velocity Path')
  set(gca,'FontSize',15);
end

figure('WindowState', 'maximized')
if columns==2
  plot(Aj(1,:),Aj(2,:),'LineWidth',3) 
  grid on
  xlabel('D_1')
  ylabel('D_2')
  title('Acceleration Path')
  set(gca,'FontSize',15);
elseif columns==3
  plot3(Aj(1,:),Aj(2,:),Aj(3,:),'LineWidth',3) 
  grid on
  xlabel('D_1')
  ylabel('D_2')
  zlabel('D_3')
  title('Acceleration Path')
  set(gca,'FontSize',15);
%}
end