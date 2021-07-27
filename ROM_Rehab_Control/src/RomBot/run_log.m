test1_data = textread("example.txt",'','headerlines',2);
t = linspace(1,5,size(test1_data,1));

theta = test1_data(:,1); 
r = test1_data(:,2); 


figure(1) 
plot(t,theta); 
title("Time vs Theta")
xlabel("Time [s]") 
ylabel("Position [deg]") 

figure(2) 
plot(t,r); 
title('Time vs r')
xlabel("Time [s]") 
ylabel("Position [cm]") 