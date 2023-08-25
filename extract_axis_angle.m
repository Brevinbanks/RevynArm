% Created 8/13/2023 By Brevin Banks
% Modified 8/20/2023 By Brevin Banks
% This function takes a 3x3 rotation matrix and extracts the theta and x
% where theta and x are the angle of rotation about an axis and x is the
% unit vector axis. These theta and x are the variables used to calculate
% the eponential form of the transform where R = e^(x*theta)
% Input
%   R - an so3 3x3 rotation matrix
% Output
%   x - unit vector rotational axis in 3D space
%   theta - angle of rotation in radians bounded between [0,2*pi]
function [x,theta] = extract_axis_angle(R)

% Ensure the matrix given is a rotation matrix
if abs(det(R)-1)>0.001
    error('The R given to extract_axis_angle did not have a determinant of 1. Improper rotation matrix given')
end

% Break out the rotation matrix to elements
R00 = R(1,1);
R01 = R(1,2);
R02 = R(1,3);
R10 = R(2,1);
R11 = R(2,2);
R12 = R(2,3);
R20 = R(3,1);
R21 = R(3,2);
R22 = R(3,3);

% Find the angle of rotation
theta = acos(( R00 + R11 + R22 - 1)/2);

% Find the axis elements in 3D of the unit vector
x1 = (R21 - R12)/sqrt((R21 - R12)^2+(R02 - R20)^2+(R10 - R01)^2);
x2 = (R02 - R20)/sqrt((R21 - R12)^2+(R02 - R20)^2+(R10 - R01)^2);
x3 = (R10 - R01)/sqrt((R21 - R12)^2+(R02 - R20)^2+(R10 - R01)^2);

% Fill elements into the vector
x = [x1,x2,x3];

% Ensure that the returned axis is a unit vector
if norm(x)~=1
    x = x./norm(x);
end

end


