
syms m g I l m_h tau theta theta_dot theta_ddot real
q = theta; 
q_dot = theta_ddot; 
q_ddot = [theta_ddot];
states = [q;q_dot]; 

gsl = [cos(q) -sin(q) 0 (l/2)*cos(q);...
        sin(q) cos(q) 0 (l/2)*sin(q);...
        0 0 1 0;...
        0 0 0 1]; 
glt =  [1 0 0 (l/2);...
        0 1 0 0;... 
        0 0 1 (h/2);... 
        0 0 0 1]; 
    
gst = gsl*glt;
    
zeta = simplify(inv(gst)*diff(gst,q)); 
wf1 = skew2angvel(zeta(1:3,1:3)); 
jacobi = [zeta(1,4);zeta(2,4);zeta(3,4);wf1]; 
mass_mat = [eye(3)* m,zeros(3);zeros(3),eye(3)*I]; 

M = jacobi.' * mass_mat * jacobi
C = mass2coriolis(M)
N = m*g*gsl(2,4) + m_h*g*gst(2,4)

