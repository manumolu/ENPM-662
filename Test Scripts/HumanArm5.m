
pi = 22/7;

%Vertical Flexion               (Shoulder)
L1 = Revolute('theta', 0,   'd', 0,    'alpha',  pi/2, 'a',    0 , 'qlim', [-pi,pi/3]);

%Abduction                       (Shoulder)
L2 = Revolute('theta', pi/2,'d', 0,    'alpha',  pi/2, 'a',     0  ,'qlim', [-pi,pi/4]);

%Horizontal Flexion              (Shoulder)
L3 = Revolute('theta', pi/2,'d', -0.4, 'alpha',  pi/2, 'a',     0  ,'qlim', [-pi/3,pi]);

%Extension/Flexion                (Elbow) 
L4 = Revolute('theta', pi,  'd', 0,    'alpha',  pi/2, 'a',     0  ,'qlim', [-5*(pi/6),0]);

%Supination/Pronation             (Elbow)
L5 = Revolute('theta', pi,  'd', -0.5, 'alpha',  pi/2, 'a',     0 , 'qlim', [-pi,pi/2]);

%Extension/Flexion                (Wrist)
L6 = Revolute('theta', pi/2,'d', 0,    'alpha',  pi/2, 'a',     0  ,'qlim', [-15*(pi/36),pi/2]);

%Radial/Ulnar Deviation           (Wrist)
L7 = Revolute('theta', 0,   'd', 0,    'alpha', -pi/2, 'a', -0.15 , 'qlim', [-pi/6,pi/9]);

r=SerialLink([L1 L2 L3 L4 L5 L6 L7]);
r.plot([0 0 0 0 0 0 0])
r.teach                           