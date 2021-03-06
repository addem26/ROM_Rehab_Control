%%
%Setup symbolic variables and state space 
syms m_l m_h m_p g I_l I_p l  h tau d d_dot d_ddot theta theta_dot theta_ddot tau_control ... % For Dynamics 
     x_ee y_ee z_ee roll_ee pitch_ee yaw_ee... % Postion of EE
     x_ee_d y_ee_d z_ee_d roll_ee_d pitch_ee_d yaw_ee_d... % Desired positon of EE     
     xdot_ee ydot_ee zdot_ee rolldot_ee pitchdot_ee yawdot_ee... % Velocity of EE
     xdot_ee_d ydot_ee_d zdot_ee_d rolldot_ee_d pitchdot_ee_d yawdot_ee_d...% Desired velocity of EE
     xddot_ee yddot_ee zddot_ee rollddot_ee pitchddot_ee yawddot_ee... %Accelration of EE 
     xddot_ee_d yddot_ee_d zddot_ee_d rollddot_ee_d pitchddot_ee_d yawddot_ee_d... %Desired acceleration of EE
     f_x f_y f_z tau_x tau_y tau_z...%External Force 
     M_i B_i K_i real %Impedence gains 

q = [theta;d]; 
q_dot = [theta_dot;d_dot];
q_ddot = [theta_ddot;d_ddot];
states = [q;q_dot]; 
x = [x_ee y_ee z_ee roll_ee pitch_ee yaw_ee]'; 
x_desired = [x_ee_d y_ee_d z_ee_d roll_ee_d pitch_ee_d yaw_ee_d]'; 

xdot = [xdot_ee ydot_ee zdot_ee rolldot_ee pitchdot_ee yawdot_ee]'; 
xdot_desired = [xdot_ee_d ydot_ee_d zdot_ee_d rolldot_ee_d pitchdot_ee_d yawdot_ee_d]';

xddot_desired = [xddot_ee_d yddot_ee_d zddot_ee_d rollddot_ee_d pitchddot_ee_d yawddot_ee_d]'; 

f_ext = [f_x f_y f_z tau_x tau_y tau_z]'; 
A_lin = jacobian(f,states);
B_lin = jacobian(f,inputs);
%Define Impedence Terms 
M_d = eye(6)*M_i;
B_d = eye(6)* B_i; 
K_d = eye(6)*K_i; 

%%
%Homogenous transformation matrices from the stationary frame
%the to the arm COM and from the arm COM to the tool frame 
gsl = [cos(q(1)) -sin(q(1)) 0 (l/2)*cos(q(1));...
        sin(q(1)) cos(q(1)) 0 (l/2)*sin(q(1));...
          0      0    1      0;...
          0      0    0      1];
      
glp =  [1 0 0 ((l/2) + q(2)/2);...
        0 1 0       0;... 
        0 0 1       0;... 
        0 0 0       1]; 
      
gsp = gsl*glp; 

    
gpt =  [1 0 0   q(2)/2;...
        0 1 0     0;... 
        0 0 1   (h/2);... 
        0 0 0     1]; 
    
gst = simplify(gsl*glp*gpt)


%%
%Work for spatial and body jacobains
zeta_1 = simplify(diff(gst,q(1))*inv(gst));  
wf1_1 = skew2angvel(zeta_1(1:3,1:3)); 
zeta_2 = simplify(diff(gst,q(2))*inv(gst)); 
wf1_2 = skew2angvel(zeta_2(1:3,1:3)); 

sj1 = [zeta_1(1,4);zeta_1(2,4);zeta_1(3,4);wf1_1]; 
sj2 = [zeta_2(1,4);zeta_2(2,4);zeta_2(3,4);wf1_2]; 

spatial_jacobian = [sj1 sj2]

zeta_b_1_r = simplify(inv(gsl)*diff(gsl,q(1)));
wfb_1_r = skew2angvel(zeta_b_1_r(1:3,1:3)); 
zeta_b_2_r = simplify(inv(gsl)*diff(gsl,q(2)));
wfb_2_r = skew2angvel(zeta_b_2_r(1:3,1:3));
bj1_r = [zeta_b_1_r(1,4);zeta_b_1_r(2,4);zeta_b_1_r(3,4);wfb_1_r];
bj2_r = [zeta_b_2_r(1,4);zeta_b_2_r(2,4);zeta_b_2_r(3,4);wfb_2_r];

body_jacobian1 = [bj1_r bj2_r];  

zeta_b_1_p = simplify(inv(gsp)*diff(gsp,q(1)));
wfb_1_p = skew2angvel(zeta_b_1_p(1:3,1:3)); 
zeta_b_2_p = simplify(inv(gsp)*diff(gsp,q(2)));
wfb_2_p = skew2angvel(zeta_b_2_p(1:3,1:3));
bj1_p = [zeta_b_1_p(1,4);zeta_b_1_p(2,4);zeta_b_1_p(3,4);wfb_1_p];
bj2_p = [zeta_b_2_p(1,4);zeta_b_2_p(2,4);zeta_b_2_p(3,4);wfb_2_p];

body_jacobian2 = [bj1_p bj2_p]; 







mass_mat_r = [eye(3)* m_l,zeros(3);zeros(3),eye(3)*I_l];

mass_mat_p = [eye(3)* m_p,zeros(3);zeros(3),eye(3)*I_p];

%%%

%%
%Required terms for control algorithms - Mass-Inertia Tensor Matrix,Coriolis Matrix   
%Gravitational Matrix, FWD kinematics and the Spatial Jacobian and its pseudoinverse 
M = body_jacobian1.' * mass_mat_r * body_jacobian1 + body_jacobian2.' * mass_mat_p * body_jacobian2
C = mass2coriolis(M) %Hardcoded for 2-DOF
N = m_l*g*gsl(2,4) + m_p*g*(simplify(gsp(2,4))) + m_h*g*gst(2,4)  


psuedo_spatial_jacobian = simplify(pinv(spatial_jacobian));

x_vec = gst(1:3,4) %Forward Kineamtics  

%Robot Manipulator Equation 
tau = M*q_ddot + C*q_dot + N

%Impedence Control Law
tau_control = simplify(M*psuedo_spatial_jacobian*(xddot_desired + inv(M_d)* ...
                (-B_d *(xdot - xdot_desired) - K_d * (x - x_desired) - f_ext))...
                 + N - (spatial_jacobian')*f_ext)
            
            
            
            