%Robot Model for Visualization Purposes using Peter Corke's ToolBox
%Dh Parameters

dh =  [0	0	0   pi/2;  
	   pi/2	0	0	 pi/2;
	   pi/2 20	0	 pi/2;
	   pi	0	0	 pi/2;
	   pi	25	0	 pi/2;
	   pi/2	0	0	 pi/2;
	   pi	0	10	 pi/2;]
 
%Construction of Manipulator
r=SerialLink(dh)
%Plotting the Manipulator
r.plot([0 0 0 0 0 0 0 ])
r.teach

