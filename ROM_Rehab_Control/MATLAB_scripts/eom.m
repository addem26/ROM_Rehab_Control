%%
%Setup symbolic variables and state space 
syms m_l g I l m_h h tau theta theta_dot theta_ddot tau_control ... % For Dynamics 
     x_ee y_ee z_ee roll_ee pitch_ee yaw_ee... % Postion of EE
     x_ee_d y_ee_d z_ee_d roll_ee_d pitch_ee_d yaw_ee_d... % Desired positon of EE     
     xdot_ee ydot_ee zdot_ee rolldot_ee pitchdot_ee yawdot_ee... % Velocity of EE
     xdot_ee_d ydot_ee_d zdot_ee_d rolldot_ee_d pitchdot_ee_d yawdot_ee_d...% Desired velocity of EE
     xddot_ee yddot_ee zddot_ee rollddot_ee pitchddot_ee yawddot_ee... %Accelration of EE 
     xddot_ee_d yddot_ee_d zddot_ee_d rollddot_ee_d pitchddot_ee_d yawddot_ee_d... %Desired acceleration of EE
     f_x f_y f_z tau_x tau_y tau_z...%External Force 
     M_i B_i K_i real %Impedence gains 
q = theta; 
q_dot = theta_ddot; 
q_ddot = [theta_ddot];
states = [q;q_dot]; 


x = [x_ee y_ee z_ee roll_ee pitch_ee yaw_ee]'; 
x_desired = [x_ee_d y_ee_d z_ee_d roll_ee_d pitch_ee_d yaw_ee_d]'; 

xdot = [xdot_ee ydot_ee zdot_ee rolldot_ee pitchdot_ee yawdot_ee]'; 
xdot_desired = [xdot_ee_d ydot_ee_d zdot_ee_d rolldot_ee_d pitchdot_ee_d yawdot_ee_d]';

xddot_desired = [xddot_ee_d yddot_ee_d zddot_ee_d rollddot_ee_d pitchddot_ee_d yawddot_ee_d]'; 

f_ext = [f_x f_y f_z tau_x tau_y tau_z]'; 

%Define Impedence Terms 
M_d = ones(size(q))* M_i;
B_d = ones(size(q))* B_i; 
K_d = ones(size(q)) *K_i; 

%%
%Homogenous transformation matrices from the stationary frame
%the to the arm COM and from the arm COM to the tool frame 
gsl = [cos(q) -sin(q) 0 (l/2)*cos(q);...
        sin(q) cos(q) 0 (l/2)*sin(q);...
          0      0    1      0;...
          0      0    0      1]; 

glt =  [1 0 0 (l/2);...
        0 1 0   0;... 
        0 0 1 (h/2);... 
        0 0 0   1]; 
    
gst = gsl*glt; 


%%
%Work for spatial and body jacobains
zeta_s = simplify(diff(gst,q)*inv(gst));  
wf1_s = skew2angvel(zeta_s(1:3,1:3)); 

zeta_b = simplify(inv(gst)*diff(gst,q));
wf1 = skew2angvel(zeta_b(1:3,1:3)); 
body_jacobian = [zeta_b(1,4);zeta_b(2,4);zeta_b(3,4);wf1];
mass_mat = [eye(3)* m_l,zeros(3);zeros(3),eye(3)*I]; 
%%%

%%
%Required terms for control algorithms - Mass-Inertia Tensor Matrix,Coriolis Matrix   
%Gravitational Matrix, FWD kinematics and the Spatial Jacobian and its pseudoinverse 

M = body_jacobian.' * mass_mat * body_jacobian
C = mass2coriolis(M) %Hardcoded for 1-DOF
N = m_l*g*gsl(2,4) + m_h*g*gst(2,4)

spatial_jacobian = [zeta_s(1,4);zeta_s(2,4);zeta_s(3,4);wf1_s]
psuedo_spatial_jacobian = pinv(spatial_jacobian)

x_vec = gst(1:3,4) %Forward Kineamtics  

%Robot Manipulator Equation 
tau = M*q_ddot + C*q_dot + N

%Impedence Control Law
tau_control = M*psuedo_spatial_jacobian*(xddot_desired + inv(M_d)* ...
                (-B_d *(xdot - xdot_desired) - K_d * (x - x_desired) - f_ext))...
                + C + N - (spatial_jacobian')*f_ext
            
            
            
            