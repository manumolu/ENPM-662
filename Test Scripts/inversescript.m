L1 = Revolute('d', 0.6, 'alpha', -pi/2, 'a', 0);% units are in m and radians
L2 = Link('theta', 0, 'alpha', pi/2, 'a', 0, 'qlim', [-0.4,0.4], 'prismatic');
L3 = Link('theta', 0, 'alpha', 0, 'a', 0, 'qlim', [-0.5,0.2], 'prismatic');
L4 = Revolute('d', 0.5, 'alpha', pi/2, 'a', 0);
L5 = Revolute('d', 0, 'alpha', -pi/2, 'a', 0);
L6 = Revolute('d', 0.8, 'alpha', pi/2, 'a', 0);
L7 = Revolute('d', 0, 'alpha', -pi/2, 'a', 0);
L8 = Revolute('d', 0.1, 'alpha', 0, 'a', 0);
r=SerialLink([L1 L2 L3 L4 L5 L6 L7 L8]);
% T=r.fkine([0,0,0,0,0,0,0,0])
theta = pi;
v= [1 0 0];
u = [1, 0.5, 0.7];
R = angvec2r(theta, v);
T = transl(u);
T(1:3,1:3)=R
q=r.ikcon(T)