body1 = rigidBody('body1'); 
jnt1 = rigidBodyJoint('jnt1','revolute'); 
tform = trvec2tform([0.25 0.25 0]); %User defined 
setFixedTransform(jnt1,tform); 
body1.Joint = jnt1; 

robo = rigidBodyTree; 
addBody(robo,body1,'base'); 
showdetails(robo) 

show(robo)  

