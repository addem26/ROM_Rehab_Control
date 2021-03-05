%%
%Setup symbolic variables and state space 
syms m g I l m_h h tau theta theta_dot theta_ddot real
q = theta; 
q_dot = theta_ddot; 
q_ddot = [theta_ddot];
states = [q;q_dot]; 

%%
%Homogenous transformation matrices from the stationary frame
%the to the arm COM and from the arm COM to the tool frame 
gsl = [cos(q) -sin(q) 0 (l/2)*cos(q);...
        sin(q) cos(q) 0 (l/2)*sin(q);...
        0 0 1 0;...
        0 0 0 1]; 
glt =  [1 0 0 (l/2);...
        0 1 0 0;... 
        0 0 1 (h/2);... 
        0 0 0 1]; 
    
gst = gsl*glt; 


%%
%Work for spatial and body jacobains
zeta_s = simplify(diff(gst,q)*inv(gst));  
wf1_s = skew2angvel(zeta_s(1:3,1:3)); 

zeta_b = simplify(inv(gst)*diff(gst,q));
wf1 = skew2angvel(zeta_b(1:3,1:3)); 
body_jacobian = [zeta_b(1,4);zeta_b(2,4);zeta_b(3,4);wf1];
mass_mat = [eye(3)* m,zeros(3);zeros(3),eye(3)*I]; 
%%%

%%
%Required terms for control algorithms - Mass-Inertia Tensor Matrix,Coriolis Matrix   
%Gravitational Matrix, FWD kinematics and the Spatial Jacobian and its pseudoinverse 

M = body_jacobian.' * mass_mat * body_jacobian; 
C = mass2coriolis(M); %Hardcoded for 1-DOF
N = m*g*gsl(2,4) + m_h*g*gst(2,4); 

spatial_jacobian = [zeta_s(1,4);zeta_s(2,4);zeta_s(3,4);wf1_s]; 
psedo_spatial_jacobian = pinv(spatial_jacobian); 

x_vec = gst(1:3,4); %Forward Kineamtics  

