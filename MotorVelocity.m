%%
% 11/28/2018
% Ethan Stiles, Jerome Suminski, Richard Lavallee
%%
% Position of center of the ball from the plane of reference (ground)
% Velocity of center of ball as a vector
% Angular velocity of motor
% 
%%
% Pb = Position of center of the ball from ground
% Vb = Velocity of center of ball wrt world
% 
%% Variables %%
% a, x-coordinate of the center of the sphere
% b, y-coordinate of the center of the sphere
% c, z-coordinate of the center of the sphere
% av, x component of velocity of the center of the sphere
% bv, y component of velocity of the center of the sphere
% cv, z component of velocity of the center of the sphere
% xyz1, x,y and z component of the location of the first wheel wrt center
% of sphere
% xyz2, x,y and z component of the location of the second wheel wrt center
% of sphere
% xyz3, x,y and z component of the location of the third wheel wrt center
% of sphere
function [VelMagnitudes] = MotorVelocity(a,b,c,av,bv,cv)
clf
% Individual components of vectors from motors 1,2,3 
% use standard unit circle split 3 ways to find a basic xyz placement of
% each motor assembly.
xyz1 = [0,-4,4];
xyz2 = [cos(5*pi/6)*4,sin(5*pi/6)*4,4];
xyz3 = [cos(pi/6)*4,sin(pi/6)*4,4];

% Take xyz vectors, break down into components and apply to system

x1 = xyz1(1);
x2 = xyz2(1);
x3 = xyz3(1);
y1 = xyz1(2);
y2 = xyz2(2);
y3 = xyz3(2);
z1 = xyz1(3);
z2 = xyz2(3);
z3 = xyz3(3);

% Position of center of ball from ground
Pb = [a,b,c];

% Velocity of center of ball wrt world
Vc = [av,bv,cv];

% Angular velocity of ball
wb = cross(Pb,Vc);

% Position of motor 1,2,3 wrt center of ball
rm1 = [x1-a,y1-b,z1-c]; % Need to revisit
rm2 = [x2-a,y2-b,z2-c]; % Need to revisit  a, b, c values for getting motors
rm3 = [x3-a,y3-b,z3-c]; % Need to revisit  with respect to the center of ball

% Velocity of motor 1,2,3 wrt world
Vm1 = cross(rm1,wb);
Vm2 = cross(rm2,wb);
Vm3 = cross(rm3,wb);

% Angular velocty of motor 1,2,3
r = 4; %radius of ball
omg1 = Vm1*r; 
omg2 = Vm2*r;
omg3 = Vm3*r;

% Angular acceleration of motor 1,2,3 wrt wheel

a1 = omg1.^2 *r;
a2 = omg2.^2 *r;
a3 = omg3.^2 *r;

VelMagnitudes = [Vm1;Vm2;Vm3];
AccMagnitudes = [a1;a2;a3];

% Plot the velocity path of motor 1
hold on
quiver3(x1,y1,z1,Vm1(1),Vm1(2),Vm1(3))
% Plot the velocity path of motor 2
quiver3(x2,y2,z2,Vm2(1),Vm2(2),Vm2(3))

% Plot the velocity path of motor 3
quiver3(x3,y3,z3,Vm3(1),Vm3(2),Vm3(3))
title('Velocity')
xlabel('x')
ylabel('y')
zlabel('z')
legend('Velocity at Wheel 1','Velocity at Wheel 2','Velocity at Wheel 3')
end