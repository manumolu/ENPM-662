% Setting the Variables

syms a1 a2 a3 d1 d2 d3 s1 s2 s3 c1 c2 c3 ;
syms T1 T2 T3;
syms A1 A2 A3;

%Setting the A-Matrices

A1=  [c1,0,-s1,0;
      s1,0,c1,0;
      0,-1,0,d1;
      0,0,0,1];
A2=  [c2,-s2,0,-a2*c2;
      s2,c2,0,a2*s2;
      0,0,1,0;
      0,0,0,1];
A3=  [c3,-s3,0,a3*c3;
     s3,c3,0,a3*s3;
     0,0,1,0;
    0,0,0,1];

%Finding the T-Matrices

T1=A1;
T2=A1*A2;
T3=A1*A2*A3;


