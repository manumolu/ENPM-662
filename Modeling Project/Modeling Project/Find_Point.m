%AIM: Generate a Random Point in the workspace to use for inverse
%Kinematics

%You find a pose as well, but I ignore it and then find it by using
%'HumanArmInverseKinematics.m '

syms c1 c2 c3 c4 c5 c6 c7 
syms s1 s2 s3 s4 s5 s6 s7
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
syms T7

theta1 = -3.14/6 
theta2 = 0 
theta3 = 3.14/6 
theta4 = -3.14/2 
theta5 = 0
theta6 = -3.14/6 
theta7 = -3.14/6 

c1 = cos(theta1)
c2 = cos(theta2)
c3 = cos(theta3)
c4 = cos(theta4)
c5 = cos(theta5)
c6 = cos(theta6)
c7 = cos(theta7)

s1 = sin(theta1)
s2 = sin(theta2)
s3 = sin(theta3)
s4 = sin(theta4)
s5 = sin(theta5)
s6 = sin(theta6)
s7 = sin(theta7)


 
 T7 =[   c7*(s6*(s4*(c3*s1 + c1*s2*s3) - c1*c2*c4) + c6*(s5*(s1*s3 - c1*c3*s2) - c5*(c4*(c3*s1 + c1*s2*s3) + c1*c2*s4))) + s7*(s5*(c4*(c3*s1 + c1*s2*s3) + c1*c2*s4) + c5*(s1*s3 - c1*c3*s2)), s6*(s5*(s1*s3 - c1*c3*s2) - c5*(c4*(c3*s1 + c1*s2*s3) + c1*c2*s4)) - c6*(s4*(c3*s1 + c1*s2*s3) - c1*c2*c4), c7*(s5*(c4*(c3*s1 + c1*s2*s3) + c1*c2*s4) + c5*(s1*s3 - c1*c3*s2)) - s7*(s6*(s4*(c3*s1 + c1*s2*s3) - c1*c2*c4) + c6*(s5*(s1*s3 - c1*c3*s2) - c5*(c4*(c3*s1 + c1*s2*s3) + c1*c2*s4))), 50*s4*(c3*s1 + c1*s2*s3) - 40*c1*c2 + s5*(c4*(c3*s1 + c1*s2*s3) + c1*c2*s4) - 15*c7*(s6*(s4*(c3*s1 + c1*s2*s3) - c1*c2*c4) + c6*(s5*(s1*s3 - c1*c3*s2) - c5*(c4*(c3*s1 + c1*s2*s3) + c1*c2*s4))) - 15*s7*(s5*(c4*(c3*s1 + c1*s2*s3) + c1*c2*s4) + c5*(s1*s3 - c1*c3*s2)) + c5*(s1*s3 - c1*c3*s2) - 50*c1*c2*c4 - 9/2;
        - s7*(s5*(c4*(c1*c3 - s1*s2*s3) - c2*s1*s4) + c5*(c1*s3 + c3*s1*s2)) - c7*(s6*(s4*(c1*c3 - s1*s2*s3) + c2*c4*s1) + c6*(s5*(c1*s3 + c3*s1*s2) - c5*(c4*(c1*c3 - s1*s2*s3) - c2*s1*s4))), c6*(s4*(c1*c3 - s1*s2*s3) + c2*c4*s1) - s6*(s5*(c1*s3 + c3*s1*s2) - c5*(c4*(c1*c3 - s1*s2*s3) - c2*s1*s4)), s7*(s6*(s4*(c1*c3 - s1*s2*s3) + c2*c4*s1) + c6*(s5*(c1*s3 + c3*s1*s2) - c5*(c4*(c1*c3 - s1*s2*s3) - c2*s1*s4))) - c7*(s5*(c4*(c1*c3 - s1*s2*s3) - c2*s1*s4) + c5*(c1*s3 + c3*s1*s2)),       15*s7*(s5*(c4*(c1*c3 - s1*s2*s3) - c2*s1*s4) + c5*(c1*s3 + c3*s1*s2)) - 50*s4*(c1*c3 - s1*s2*s3) - 40*c2*s1 - s5*(c4*(c1*c3 - s1*s2*s3) - c2*s1*s4) + 15*c7*(s6*(s4*(c1*c3 - s1*s2*s3) + c2*c4*s1) + c6*(s5*(c1*s3 + c3*s1*s2) - c5*(c4*(c1*c3 - s1*s2*s3) - c2*s1*s4))) - c5*(c1*s3 + c3*s1*s2) - 50*c2*c4*s1;
                                                                             s7*(s5*(s2*s4 - c2*c4*s3) + c2*c3*c5) - c7*(s6*(c4*s2 + c2*s3*s4) + c6*(c5*(s2*s4 - c2*c4*s3) - c2*c3*s5)),                                              c6*(c4*s2 + c2*s3*s4) - s6*(c5*(s2*s4 - c2*c4*s3) - c2*c3*s5),                                                                           c7*(s5*(s2*s4 - c2*c4*s3) + c2*c3*c5) + s7*(s6*(c4*s2 + c2*s3*s4) + c6*(c5*(s2*s4 - c2*c4*s3) - c2*c3*s5)),                                                                                                                           s5*(s2*s4 - c2*c4*s3) - 40*s2 - 50*c4*s2 - 15*s7*(s5*(s2*s4 - c2*c4*s3) + c2*c3*c5) + 15*c7*(s6*(c4*s2 + c2*s3*s4) + c6*(c5*(s2*s4 - c2*c4*s3) - c2*c3*s5)) + c2*c3*c5 - 50*c2*s3*s4 - 9/2;
                                                                                                                                                                                      0,                                                                                                          0,                                                                                                                                                                                    0,                                                                                                                                                                                                                                                                                                                    1]
 