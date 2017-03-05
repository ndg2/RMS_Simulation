function thetas = robot_ik_ng(x, y, z)
%% INVERSE KINEMATICS
% Calculates the full inverse kinematics for the portrait-drawing robot.
%
% The three input arguments (x, y, z) are the desired coordinates of the
% robot's end-effector tip in mm, specified in the base frame.
%
%     x: x-coordinate of the origin of frame 3 in frame 0, in mm
%     y: y-coordinate of the origin of frame 3 in frame 0, in mm
%     z: z-coordinate of the origin of frame 3 in frame 0, in mm
%
% The output (thetas) is a matrix that contains the joint angles needed to
% place the robot's end-effector at the desired position. The first row is
% theta1, the second row is theta2, etc.,
% The number of columns is the number of inverse kinematics solutions that
% were found. Joint angles are specified in radians.
%  If this function cannot find a solution to the IK problem, it will pass
% back NaN (not a number) for all of the thetas.

%% CHECK INPUTS

if (nargin < 3)
    error('Not enough input arguments.  You need three.')
elseif (nargin == 3)
    % Correct
elseif (nargin > 3)
    error('Too many input arguments.  You need three.')
end

%% CALCULATE INVERSE KINEMATICS SOLUTION(S)

% Dimensions
a =   57; % mm
b =  142; % mm
c =   10; % mm
d =  142; % mm
e = 47-z; % mm

% End-effector position
o30=[x y z]';

o30x=o30(1);
o30y=o30(2);
o30z=o30(3);

% Inverse Kinematics by geometry
r=sqrt(o30x^2+o30y^2);
theta2_a=acos((r^2-b^2-d^2)/(2*b*d));
theta2_b=-theta2_a;

beta=atan2(y,x);
sigma_a=atan2(d/r*sin(theta2_a),(r^2+b^2-d^2)/(2*r*b));
sigma_b=atan2(d/r*sin(theta2_b),(r^2+b^2-d^2)/(2*r*b));

theta1_a=beta-sigma_a;
theta1_b=beta-sigma_b;

th1=[theta1_a theta1_b];
th2=[theta2_a theta2_b];

% If not within workspace, return NaN
if x^2+y^2>(b+d)^2
    th1 = NaN;
    th2 = NaN;
    z = NaN;
    thetas = [th1; th2;z];
    return
end

%% FORMAT OUTPUT

thetas = [th1; th2;z,z];

end
