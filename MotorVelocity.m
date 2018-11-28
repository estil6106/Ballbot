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
%%
function [Magnitudes] = MotorVelocity(a,b,c,av,bv,cv,xyz1,xyz2,xyz3)
% Individual components of vectors from motors 1,2,3 
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
wb = cross(Pb,inv(Vc));

% Position of motor 1,2,3 wrt center of ball
rm1 = [x1,y1,z1];
rm2 = [x2,y2,z2];
rm3 = [x3,y3,z3];

% Velocity of motor 1,2,3 wrt world
Vm1 = cross(rm1,wb);
Vm2 = cross(rm2,wb);
Vm3 = cross(rm3,wb);

Magnitudes = [Vm1;Vm2;Vm3];
end