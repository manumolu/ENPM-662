clear;

dhparams = [0	0	0   pi/2;  
	   pi/2	0	0	 pi/2;
	   pi/2 0.02	0	 pi/2;
	   pi	0	0	 pi/2;
	   pi	0.025	0	 pi/2;
	   pi/2	0	0	 pi/2;
	   pi	0	0.005	 pi/2;] 
   
robot = robotics.RigidBodyTree()

% Link 1 
link1 = robotics.RigidBody('L1');
jnt1 = robotics.Joint('jnt1','revolute');
jnt1.PositionLimits = deg2rad([-120, 120]);
setFixedTransform(jnt1,dhparams(1,:),'dh');
jnt1.JointAxis = [1 0 0];
link1.Joint = jnt1;
addBody(robot, link1, 'base');

% Link 2
link2 = robotics.RigidBody('L2');
jnt2 = robotics.Joint('jnt2','revolute');
jnt2.PositionLimits = deg2rad([-90, 120]);
setFixedTransform(jnt2,dhparams(2,:),'dh');
jnt2.JointAxis = [0 1 0];
link2.Joint = jnt2;
addBody(robot, link2, 'L1');

%Link 3

link3 = robotics.RigidBody('L3');
jnt3 = robotics.Joint('jnt3','revolute');
jnt3.PositionLimits = deg2rad([-60, 180]);
setFixedTransform(jnt3,dhparams(3,:),'dh');
jnt3.JointAxis = [0 0 1];
link3.Joint = jnt3;
addBody(robot, link3, 'L2');

%Link 4
link4 = robotics.RigidBody('L4');
jnt4 = robotics.Joint('jnt4','revolute');
jnt4.PositionLimits = deg2rad([-60, 180]);
setFixedTransform(jnt4,dhparams(4,:),'dh');
jnt4.JointAxis = [1 0 0];
link4.Joint = jnt4;
addBody(robot, link4, 'L3');

%Link 5

link5 = robotics.RigidBody('L5');
jnt5 = robotics.Joint('jnt5','revolute');
jnt5.PositionLimits = deg2rad([-20, 20]);
setFixedTransform(jnt5,dhparams(5,:),'dh');
jnt5.JointAxis = [0 0 1];
link5.Joint = jnt5;
addBody(robot, link5, 'L4');

%Link 6

link6 = robotics.RigidBody('L6');
jnt6 = robotics.Joint('jnt6','revolute');
jnt6.PositionLimits = deg2rad([-45, 45]);
setFixedTransform(jnt6,dhparams(5,:),'dh');
jnt6.JointAxis = [1 0 0];
link6.Joint = jnt6;
addBody(robot, link6, 'L5');

% Link 7
link7 = robotics.RigidBody('L7');
jnt7 = robotics.Joint('jnt7','revolute');
jnt7.PositionLimits = deg2rad([-5, 5]);
setFixedTransform(jnt7,dhparams(5,:),'dh');
jnt7.JointAxis = [0 1 0];
link7.Joint = jnt7;
addBody(robot, link7, 'L6');

% Structural Details
showdetails(robot)

%Base Configuration
Qhome = robot.homeConfiguration
qhome = [Qhome.JointPosition]
figure;
show(robot,Qhome);

%Random Configuration 1
Q = randomConfiguration(robot);
q = [Q.JointPosition]
T = getTransform(robot,Q,'L7')
eulT=tform2eul(T)
figure;
show(robot,Q);

%Random Configuration 2
%J = randomConfiguration(robot);
%j = [J.JointPosition]
%R = getTransform(robot,J,'L7')
%eulJ=tform2eul(R)
%figure;
%show(robot,J);


