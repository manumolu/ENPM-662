dhparams = [0	0	0   pi/2;  
	   pi/2	0	0	 pi/2;
	   pi/2 0.2	0	 pi/2;
	   pi	0	0	 pi/2;
	   pi	0.25	0	 pi/2;
	   pi/2	0	0	 pi/2;
	   pi	0	0.05	 pi/2;]
   
robot = robotics.RigidBodyTree()

% Link 1 
link1 = robotics.RigidBody('L1');
jnt1 = robotics.Joint('jnt1','revolute');
jnt1.PositionLimits = deg2rad([-180, 180]);
setFixedTransform(jnt1,dhparams(1,:),'dh');
jnt1.JointAxis = [0 0 1];
link1.Joint = jnt1;
addBody(robot, link1, 'base');

% Link 2
link2 = robotics.RigidBody('L2');
jnt2 = robotics.Joint('jnt2', 'revolute');
jnt2.PositionLimits = deg2rad([0,180]);
setFixedTransform(jnt2,dhparams(2,:),'dh');
jnt2.JointAxis = [0,1,0];
link2.Joint = jnt2;
addBody(robot, link2, 'L1');

%Link 3
link3 = robotics.RigidBody('L3');
jnt3 = robotics.Joint('jnt3','revolute');
jnt3.PositionLimits = deg2rad([0,130]);
setFixedTransform(jnt3,dhparams(3,:),'dh');
jnt3.JointAxis = [1 0 0];
link3.Joint = jnt3;
addBody (robot, link3, 'L2');

%End Effector
link4 = robotics.RigidBody('L4');
jnt4 = robotics.Joint('jnt4','fixed');
setFixedTransform(jnt4, [eye(3),[ThreeLinkRobot.l3;0;0];[0,0,0,1]]);
link4.Joint = jnt4;
addBody(robot, link4, 'L3');


% Structural Details
showdetails(robot);

%Base Configuration
Qhome = robot.homeConfiguration
qhome = [Qhome.JointPosition]
figure;
show(robot,Qhome);

%Random Configuration 1
Q = randomConfiguration(robot);
q = [Q.JointPosition]
T = getTransform(robot,Q,'L4')
eulT=tform2eul(T)
figure;
show(robot,Q);

%Random Configuration 2
J = randomConfiguration(robot);
j = [J.JointPosition]
R = getTransform(robot,J,'L4')
eulJ=tform2eul(R)
figure;
show(robot,J);





























