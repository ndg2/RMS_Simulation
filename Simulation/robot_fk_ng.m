function [points_to_plot, x30, y30, z30] = robot_fk_ng(theta1, theta2,z)
%
% Given the two joint angles, this function returns the Cartesian
% coordinates of points along the robot as well as the Cartesian
% coordinates for the starting and ending points of vectors along the x, y,
% and z axes of the robot's end-effector frame.

%% ROBOT PARAMETERS

a =  57; % mm
b = 142; % mm
c =  10; % mm
d = 142; % mm
e =  47-z; % mm

% Length of coordinate frame vectors, in mm.
vlen = 25;

%% DH MATRICES

A01 = dh_ng(0,     0,    a, theta1);
A12 = dh_ng(b,     0,   -c, theta2);
A23 = dh_ng(d,     0,   -e, 0);

%% ORIGIN POSITIONS

% Origin in homogenous representation
o = [0 0 0 1]';

% Calculate the origin of each frame
o0 = o;
o1 = A01*o;
extra_point1 = A01*[b 0 0 1]';
o2 = A01*A12*o;
extra_point2 = A01*A12*[b 0 0 1]';
o3 = A01*A12*A23*o;

% Put origin points together for plotting.
points_to_plot = [o0 o1 extra_point1 o2 extra_point2 o3];

%% END-EFFECTOR COORDINATE FRAME 

% Save the full transformation
T30 = A01*A12*A23;

% X, Y, and Z unit  vectors of frame 3 in frame 0
x30 = [o3 (T30 * [vlen 0 0 1]')];
y30 = [o3 (T30 * [0 vlen 0 1]')];
z30 = [o3 (T30 * [0 0 vlen 1]')];
