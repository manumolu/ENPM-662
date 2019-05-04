x%Forward Verification using inverse kinematic values for a desired point
%and computing the jacobian matrix for that case as well. (Numerical
%Values)

syms c1 c2 c3 c4 c5 c6 c7 
syms s1 s2 s3 s4 s5 s6 s7
syms A1 A2 A3 A4 A5 A6 A7
syms T1 T2 T3 T4 T5 T6 T7
syms O1 O2 O3 O4 O5 O6 O7
syms Z1 Z2 Z3 Z4 Z5 Z6 Z7
syms J1 J2 J3 J4 J5 J6 J7
syms O0 Z0
syms jacobian
syms qdot
syms q

%The Angles gotten from the inverse kinematics
q = [-0.6872;-3.690;1.3082;2.8797;-4.0210;5.2227;3.2229]

%Initliazing values for c1 c2...etc.
c1 = cos(-2.1481);
c2 = cos(-0.5862);
c3 = cos(-0.0515);
c4 = cos(0.9492);
c5 = cos(-0.6006);
c6 = cos(6.1874);
c7 = cos( 0.2282);

s1 = sin(-2.1481);
s2 = sin(-0.5862);
s3 = sin(-0.0515 );
s4 = sin(0.9492);
s5 = sin(-0.6006);
s6 = sin(6.1874);
s7 = sin(0.2282);

%Setting the A-Matrices
A1 = [c1,0,s1,-4.5;
     s1,0,-c1,0;
     0,1,0,-4.5;
     0,0,0,1];
 
A2 = [-s2,0,c2,0;
      c2,0,s2,0;
      0,1,0,0;
      0,0,0,1];

A3 = [-s3,0,c3,0;
      c3,0,s3,0;
      0,1,0,-40;
      0,0,0,1];
  
A4 = [-c4,0,-s4,0;
      -s4,0,c4,0;
      0,1,0,0;
      0,0,0,1];
  
A5 = [-c5,0,-s5,0;
      -s5,0,c5,0;
      0,1,0,-50;
      0,0,0,1];
  
A6 = [-c6,0,-s6,0;
      -s6,0,c6,0;
      0,1,0,1;
      0,0,0,1];
  
A7 =[c7,0,-s7,-15*c7;
     s7,0,c7,-15*s7;
     0,1,0,0;
     0,0,0,1];
 
 %Setting the T_matrices
T1=A1;
T2=A1*A2;
T3=A1*A2*A3;
T4=A1*A2*A3*A4;
T5=A1*A2*A3*A4*A5;
T6=A1*A2*A3*A4*A5*A6;
T7 = A1*A2*A3*A4*A5*A6*A7


%Finding the Origins after each Homogenoous Tranformation

O0= [-4.5,0,-4.5];                  % Standard Origin Computed after analyzing simulink body
O1= [T1(1,4),T1(2,4),T1(3,4)];
O2= [T2(1,4),T2(2,4),T2(3,4)];
O3= [T3(1,4),T3(2,4),T3(3,4)];
O4= [T4(1,4),T4(2,4),T4(3,4)];
O5= [T5(1,4),T5(2,4),T5(3,4)];
O6= [T6(1,4),T6(2,4),T6(3,4)];
O7= [T7(1,4),T7(2,4),T7(3,4)];        %(x,y,z of end-effector)

%Finding the Z-Axis Values 
Z0= [-4.5,0,-5.5];                  %Z0 Computed 
Z1= [T1(1,3),T1(2,3),T1(3,3)];
Z2= [T2(1,3),T2(2,3),T2(3,3)];
Z3= [T3(1,3),T3(2,3),T3(3,3)];
Z4= [T4(1,3),T4(2,3),T4(3,3)];
Z5= [T5(1,3),T5(2,3),T5(3,3)];
Z6= [T6(1,3),T6(2,3),T6(3,3)];
Z7= [T7(1,3),T7(2,3),T7(3,3)];
 
%Finding the Velocity Jacobian
J1 = cross(Z0,(O7-O0));
J2 = cross(Z1,(O7-O1));
J3 = cross(Z2,(O7-O2));
J4 = cross(Z3,(O7-O3));
J5 = cross(Z4,(O7-O4));
J6 = cross(Z5,(O7-O5));
J7 = cross(Z6,(O7-O6));

%Constructing the Jacobian
jacobian= [J1.',J2.',J3.',J4.',J5.',J6.',J7.';Z0.',Z1.',Z2.',Z3.',Z4.',Z5.',Z6.']

%Finding the end-effector velocities.
qdot = jacobian*q
