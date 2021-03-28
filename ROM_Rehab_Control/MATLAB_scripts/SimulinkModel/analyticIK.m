function [theta,d] = analyticIK(x,y,l) 

theta = atan2(y,x); 

if theta == -pi/2 
    d = y; 
else 
    d = (x/cos(theta)) - l; 
end 

end 