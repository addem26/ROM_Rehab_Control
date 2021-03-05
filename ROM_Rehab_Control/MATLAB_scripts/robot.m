l = 0.5; %Define 
base_link = rigidBody('base_link'); 
link = rigidBody('link');
handle = rigidBody('handle'); 

jnt1 = rigidBodyJoint('jnt1','revolute');
jnt2 = rigidBodyJoint('jnt2','fixed');
jnt3 = rigidBodyJoint('jnt3', 'fixed');

tform1 = trvec2tform([0.0 0.00 l/6]); %User defined
tform2 = trvec2tform([l l 0]); 
tform3 = trvec2tform([0 0 l/2]); 

setFixedTransform(jnt1,tform1); 
setFixedTransform(jnt2,tform2); 
setFixedTransform(jnt3,tform3)

base_link.Joint = jnt1; 
link.Joint = jnt2;
handle.Joint = jnt3; 

link.CenterOfMass = [l l/4 l]; %Center of link  

robo = rigidBodyTree; 
addBody(robo,base_link,'base');
addBody(robo,link,'base_link');
addBody(robo,handle,'link'); 
showdetails(robo) 

show(robo)  

