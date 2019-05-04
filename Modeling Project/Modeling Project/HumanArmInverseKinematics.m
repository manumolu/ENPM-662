%Finding the Inverse Kinematics

%For Inverse Kinematics
%Given Input (x ,y, z)
%Find normalized vector for (x,y,z) from (0,0,0)
%Find position vector with respect to home configuration (
%Find Rotation Matrix using the normalized position vectors
%Plug Rotation Matrix in Transformation Matrix
%Solve system of equations for cos(theta(1))........cos(theta(6)) values. 

syms TotalT
%Home Configuration (Normalized):
Home = [(-4.5/11.884),0,(11/11.884)];

%Give Input say (-26, 64, 31)
%Point (-21.5, 64, 35.5)
%Normalized Vector is 
New_Point = [-21.5/74.5,64/74.5,35.5/74.5];


%Finding the Rotation Matrix Linking the Home Configuration to the Desired
%Configuration.
[q] = vrrotvec(Home,New_Point);
Rot_mat = vrrotvec2mat(q);

%Total Transformation Matrix
TotalT = [Rot_mat,[-21.5;64;35.5];0,0,0,1] %Offset due to position of shoulder

%Initializing the Inverse Kinematics Output Vector
fun = @IKine;
theta0 = [0,0,0,0,0,0,0];


%Solve for theta for 
%theta1 [-pi,pi/3]                %Vertical Flexion         (Shoulder)
%theta2 [-pi,pi/4]                %Abduction                (Shoulder)
%theta3 [-pi/3,pi]                %Horizontal Flexion       (Shoulder)
%theta4 [-5*(pi/6),0]             %Extension/Flexion         (Elbow) 
%theta5 [-pi,pi/2]                %Supination/Pronation      (Elbow) 
%theta6 [-15*(pi/36),pi/2]        %Extension/Flexion         (Wrist)
%theta7 [-pi/6,pi/9]              %Radial/Ulnar Deviation    (Wrist)
  
%if above theta values dont fall in the limits, point cannot be reached
%with that pose. 
theta = fsolve(fun,theta0)

if (theta(1)>= -pi && theta(1)<= pi/3)
    theta(1) = theta(1)
else disp('Not within Joint Limits')
    
end

if (theta(2)>= -pi && theta(2)<= pi/4)
    theta(2) = theta(2)
else disp('Not within Joint Limits')
end

if (theta(3)>= -pi/3 && theta(3)<= pi)
    theta(3) = theta(3)
else disp('Not within Joint Limits')
    
end
if (theta(4)>= -5*pi/6 && theta(4)<= 0)
    theta(4) = theta(4)
else disp('Not within Joint Limits')
end

if (theta(5)>= -pi && theta(5)<= pi/2)
    theta(5) = theta(5)
else disp('Not within Joint Limits')
end

if (theta(6)>= -15*pi/36 && theta(6)<= pi/2)
    theta(6) = theta(6)
else disp('Not within Joint Limits')
end

if (theta(7)>= -pi/6 && theta(7)<= pi/9)
    theta(7) = theta(7)
else disp('Not within Joint Limits')
end


  
%Simultaenous Nonlinear Equation Solver for the Forward and Computed
%Transformation MAtrices. As it is not included in the fucntion, the
%computed values have to be entered manually for every desired input. 
function F = IKine(theta)

%Equation 1 T7(1,1) = 0.9146
 F(1)= cos(theta(7))*(sin(theta(6))*(sin(theta(4))*(cos(theta(3))*sin(theta(1)) + cos(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(1))*cos(theta(2))*cos(theta(4))) + cos(theta(6))*(sin(theta(5))*(sin(theta(1))*sin(theta(3)) - cos(theta(1))*cos(theta(3))*sin(theta(2))) - cos(theta(5))*(cos(theta(4))*(cos(theta(3))*sin(theta(1)) + cos(theta(1))*sin(theta(2))*sin(theta(3))) + cos(theta(1))*cos(theta(2))*sin(theta(4))))) + sin(theta(7))*(sin(theta(5))*(cos(theta(4))*(cos(theta(3))*sin(theta(1)) + cos(theta(1))*sin(theta(2))*sin(theta(3))) + cos(theta(1))*cos(theta(2))*sin(theta(4))) + cos(theta(5))*(sin(theta(1))*sin(theta(3)) - cos(theta(1))*cos(theta(3))*sin(theta(2))))-0.9297;

%Equation 2 T7(1,2) =  0.4044 
F(2) = - sin(theta(7))*(sin(theta(5))*(cos(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(2))*sin(theta(1))*sin(theta(4))) + cos(theta(5))*(cos(theta(1))*sin(theta(3)) + cos(theta(3))*sin(theta(1))*sin(theta(2)))) - cos(theta(7))*(sin(theta(6))*(sin(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) + cos(theta(2))*cos(theta(4))*sin(theta(1))) + cos(theta(6))*(sin(theta(5))*(cos(theta(1))*sin(theta(3)) + cos(theta(3))*sin(theta(1))*sin(theta(2))) - cos(theta(5))*(cos(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(2))*sin(theta(1))*sin(theta(4))))) - 0.3604 ;

%Equation 3 T7(1,3) = 0.0031
F(3) = cos(theta(7))*(sin(theta(5))*(cos(theta(4))*(cos(theta(3))*sin(theta(1)) + cos(theta(1))*sin(theta(2))*sin(theta(3))) + cos(theta(1))*cos(theta(2))*sin(theta(4))) + cos(theta(5))*(sin(theta(1))*sin(theta(3)) - cos(theta(1))*cos(theta(3))*sin(theta(2)))) - sin(theta(7))*(sin(theta(6))*(sin(theta(4))*(cos(theta(3))*sin(theta(1)) + cos(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(1))*cos(theta(2))*cos(theta(4))) + cos(theta(6))*(sin(theta(5))*(sin(theta(1))*sin(theta(3)) - cos(theta(1))*cos(theta(3))*sin(theta(2))) - cos(theta(5))*(cos(theta(4))*(cos(theta(3))*sin(theta(1)) + cos(theta(1))*sin(theta(2))*sin(theta(3))) + cos(theta(1))*cos(theta(2))*sin(theta(4)))))-0.0758;

%Equation 4 T7(1,4) = -26.0000
F(4) = 50*sin(theta(4))*(cos(theta(3))*sin(theta(1)) + cos(theta(1))*sin(theta(2))*sin(theta(3))) - 40*cos(theta(1))*cos(theta(2)) + sin(theta(5))*(cos(theta(4))*(cos(theta(3))*sin(theta(1)) + cos(theta(1))*sin(theta(2))*sin(theta(3))) + cos(theta(1))*cos(theta(2))*sin(theta(4))) - 15*cos(theta(7))*(sin(theta(6))*(sin(theta(4))*(cos(theta(3))*sin(theta(1)) + cos(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(1))*cos(theta(2))*cos(theta(4))) + cos(theta(6))*(sin(theta(5))*(sin(theta(1))*sin(theta(3)) - cos(theta(1))*cos(theta(3))*sin(theta(2))) - cos(theta(5))*(cos(theta(4))*(cos(theta(3))*sin(theta(1)) + cos(theta(1))*sin(theta(2))*sin(theta(3))) + cos(theta(1))*cos(theta(2))*sin(theta(4))))) - 15*sin(theta(7))*(sin(theta(5))*(cos(theta(4))*(cos(theta(3))*sin(theta(1)) + cos(theta(1))*sin(theta(2))*sin(theta(3))) + cos(theta(1))*cos(theta(2))*sin(theta(4))) + cos(theta(5))*(sin(theta(1))*sin(theta(3)) - cos(theta(1))*cos(theta(3))*sin(theta(2)))) + cos(theta(5))*(sin(theta(1))*sin(theta(3)) - cos(theta(1))*cos(theta(3))*sin(theta(2))) - 50*cos(theta(1))*cos(theta(2))*cos(theta(4)) - 9/2 +21.50000;

%Equation 5 T7(2,1) = -0.2356  
F(5) = - sin(theta(7))*(sin(theta(5))*(cos(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(2))*sin(theta(1))*sin(theta(4))) + cos(theta(5))*(cos(theta(1))*sin(theta(3)) + cos(theta(3))*sin(theta(1))*sin(theta(2)))) - cos(theta(7))*(sin(theta(6))*(sin(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) + cos(theta(2))*cos(theta(4))*sin(theta(1))) + cos(theta(6))*(sin(theta(5))*(cos(theta(1))*sin(theta(3)) + cos(theta(3))*sin(theta(1))*sin(theta(2))) - cos(theta(5))*(cos(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(2))*sin(theta(1))*sin(theta(4))))) +0.2749;

%Equation 6 T7(2,2) = 0.5265
F(6) = cos(theta(6))*(sin(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) + cos(theta(2))*cos(theta(4))*sin(theta(1))) - sin(theta(6))*(sin(theta(5))*(cos(theta(1))*sin(theta(3)) + cos(theta(3))*sin(theta(1))*sin(theta(2))) - cos(theta(5))*(cos(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(2))*sin(theta(1))*sin(theta(4)))) - 0.5421;

%Equation 7 T7(2,3) = 0.8169
F(7) = sin(theta(7))*(sin(theta(6))*(sin(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) + cos(theta(2))*cos(theta(4))*sin(theta(1))) + cos(theta(6))*(sin(theta(5))*(cos(theta(1))*sin(theta(3)) + cos(theta(3))*sin(theta(1))*sin(theta(2))) - cos(theta(5))*(cos(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(2))*sin(theta(1))*sin(theta(4))))) - cos(theta(7))*(sin(theta(5))*(cos(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(2))*sin(theta(1))*sin(theta(4))) + cos(theta(5))*(cos(theta(1))*sin(theta(3)) + cos(theta(3))*sin(theta(1))*sin(theta(2)))) -0.7940;

%Equation 8 T7(2,4) = 64.0000
F(8) = 15*sin(theta(7))*(sin(theta(5))*(cos(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(2))*sin(theta(1))*sin(theta(4))) + cos(theta(5))*(cos(theta(1))*sin(theta(3)) + cos(theta(3))*sin(theta(1))*sin(theta(2)))) - 50*sin(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) - 40*cos(theta(2))*sin(theta(1)) - sin(theta(5))*(cos(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(2))*sin(theta(1))*sin(theta(4))) + 15*cos(theta(7))*(sin(theta(6))*(sin(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) + cos(theta(2))*cos(theta(4))*sin(theta(1))) + cos(theta(6))*(sin(theta(5))*(cos(theta(1))*sin(theta(3)) + cos(theta(3))*sin(theta(1))*sin(theta(2))) - cos(theta(5))*(cos(theta(4))*(cos(theta(1))*cos(theta(3)) - sin(theta(1))*sin(theta(2))*sin(theta(3))) - cos(theta(2))*sin(theta(1))*sin(theta(4))))) - cos(theta(5))*(cos(theta(1))*sin(theta(3)) + cos(theta(3))*sin(theta(1))*sin(theta(2))) - 50*cos(theta(2))*cos(theta(4))*sin(theta(1)) -64.0000;

%Equation 9 T7(3,1) = 0.3287
F(9) =  sin(theta(7))*(sin(theta(5))*(sin(theta(2))*sin(theta(4)) - cos(theta(2))*cos(theta(4))*sin(theta(3))) + cos(theta(2))*cos(theta(3))*cos(theta(5))) - cos(theta(7))*(sin(theta(6))*(cos(theta(4))*sin(theta(2)) + cos(theta(2))*sin(theta(3))*sin(theta(4))) + cos(theta(6))*(cos(theta(5))*(sin(theta(2))*sin(theta(4)) - cos(theta(2))*cos(theta(4))*sin(theta(3))) - cos(theta(2))*cos(theta(3))*sin(theta(5))))-0.2451;

%Equation 10 T7(3,2) = -0.7478
F(10) = cos(theta(6))*(cos(theta(4))*sin(theta(2)) + cos(theta(2))*sin(theta(3))*sin(theta(4))) - sin(theta(6))*(cos(theta(5))*(sin(theta(2))*sin(theta(4)) - cos(theta(2))*cos(theta(4))*sin(theta(3))) - cos(theta(2))*cos(theta(3))*sin(theta(5)))+0.7591;

%Equation 11 T7(3,3) =  0.5768
F(11) = cos(theta(7))*(sin(theta(5))*(sin(theta(2))*sin(theta(4)) - cos(theta(2))*cos(theta(4))*sin(theta(3))) + cos(theta(2))*cos(theta(3))*cos(theta(5))) + sin(theta(7))*(sin(theta(6))*(cos(theta(4))*sin(theta(2)) + cos(theta(2))*sin(theta(3))*sin(theta(4))) + cos(theta(6))*(cos(theta(5))*(sin(theta(2))*sin(theta(4)) - cos(theta(2))*cos(theta(4))*sin(theta(3))) - cos(theta(2))*cos(theta(3))*sin(theta(5)))) -0.6031;

%Equation 12 T7(3,4) = 31.0000
F(12) =  sin(theta(5))*(sin(theta(2))*sin(theta(4)) - cos(theta(2))*cos(theta(4))*sin(theta(3))) - 40*sin(theta(2)) - 50*cos(theta(4))*sin(theta(2)) - 15*sin(theta(7))*(sin(theta(5))*(sin(theta(2))*sin(theta(4)) - cos(theta(2))*cos(theta(4))*sin(theta(3))) + cos(theta(2))*cos(theta(3))*cos(theta(5))) + 15*cos(theta(7))*(sin(theta(6))*(cos(theta(4))*sin(theta(2)) + cos(theta(2))*sin(theta(3))*sin(theta(4))) + cos(theta(6))*(cos(theta(5))*(sin(theta(2))*sin(theta(4)) - cos(theta(2))*cos(theta(4))*sin(theta(3))) - cos(theta(2))*cos(theta(3))*sin(theta(5)))) + cos(theta(2))*cos(theta(3))*cos(theta(5)) - 50*cos(theta(2))*sin(theta(3))*sin(theta(4)) - 9/2 -35.5000;
 

end








