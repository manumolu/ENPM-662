% Setting the Variables

syms a1 a2 a3 a4 a5 a6 d1 d2 d3 d4 d5 d6 s1 s2 s3 s4 s5 s6 c1 c2 c3 c4 c5 c6;
syms T1 T2 T3 T4 T5 T6;
syms O0 O1 O2 O3 O4 O5 O6;
syms Z0 Z1 Z2 Z3 Z4 Z5 Z6;
syms Jv1 Jv2 Jv3 Jv4 Jv5 Jv6;
syms jacobian

%Setting the A-Matrices

A1=  [c1,0,s1,0;s1,0,-c1,0;0,1,0,d1;0,0,0,1];
A2=  [-s2,-c2,0,-a2*s2;c2,-s2,-0,a2*c2;0,0,1,0;0,0,0,1];
A3=  [c3,s3,0,a3*c3;s3,-c3,0,a3*s3;0,0,-1,0;0,0,0,1];
A4=  [-s4,0,c4,0;c4,0,-s4,0;0,-1,0,-d4;0,0,0,1];
A5=  [c5,0,s5,0;s5,0,-c5,0;0,1,0,-d5;0,0,0,1];
A6=  [c6,s6,0,0;s6,-c6,0,0;0,0,1,-d6;0,0,0,1];

%Finding the T-Matrices

T1=A1;
T2= A1*A2;
T3=A1*A2*A3;
T4=A1*A2*A3*A4;
T5=A1*A2*A3*A4*A5;
T6=A1*A2*A3*A4*A5*A6;

%Finding the Origins after each Homogenous Transformation

O0= [0,0,0];
O1= [T1(1,4),T1(2,4),T1(3,4)];
O2= [T2(1,4),T2(2,4),T2(3,4)];
O3= [T3(1,4),T3(2,4),T3(3,4)];
O4= [T4(1,4),T4(2,4),T4(3,4)];
O5= [T5(1,4),T5(2,4),T5(3,4)];
O6= [T6(1,4),T6(2,4),T6(3,4)];

% Finding the Z-Axis Values

Z0= [0,0,1]
Z1= [T1(1,3),T1(2,3),T1(3,3)]
Z2= [T2(1,3),T2(2,3),T2(3,3)]
Z3= [T3(1,3),T3(2,3),T3(3,3)]
Z4= [T4(1,3),T4(2,3),T4(3,3)]
Z5= [T5(1,3),T5(2,3),T5(3,3)]
Z6= [T6(1,3),T6(2,3),T6(3,3)]

%Finding the Velocity Jacobian

Jv1= cross(Z0,(O6-O0))
Jv2= cross(Z1,(O6-O1))
Jv3= cross(Z2,(O6-O2))
Jv4= cross(Z3,(O6-O3))
Jv5= cross(Z4,(O6-O4))
Jv6= cross(Z5,(O6-O5))
%Constructing the Jacobian Matrix

jacobian= [Jv1.',Jv2.',Jv3.',Jv4.',Jv5.',Jv6.';Z0.',Z1.',Z2.',Z3.',Z4.',Z5.']




