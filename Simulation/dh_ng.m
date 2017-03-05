function T = dh_ng(l, alpha, d, theta)

% This function calculates the homogenous matrix T(n-1 to n) for a  set of 
% DH parameters: l, alpha, d, and theta. The angles are in radians.

% l is l(n-1): distance between z(n-1) and z(n), along x(n-1)
% alpha is alpha(n-1): angle between z(n-1) and z(n), about x(n-1)
% d is d(n): distance from x(n-1) to x(n), along z(n)
% theta is theta(n): angle between x(n-1) and x(n), about z(n)

T =   [cos(theta)             -sin(theta)            0                       l;
       sin(theta)*cos(alpha)  cos(theta)*cos(alpha)  -sin(alpha) -d*sin(alpha);
       sin(theta)*sin(alpha)  cos(theta)*sin(alpha)  cos(alpha)   d*cos(alpha);
       0                      0                      0                      1];
end
