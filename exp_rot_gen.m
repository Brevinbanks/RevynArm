% Created 8/15/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function generates a 3x3 SO3 rotation matrix using an axis and an
% angle
% Created 8/13/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function will create a 3x3 rotation matrix given an axis and angle
% using the exponential form for so3 generation. R = e^(x*theta)
% See the Rodrigues' rotation formula
% Input
%   x - a unit vector axis to define the rotation about
%   theta - the angle between [-pi,pi] to rotated about
% Output
%   e_R - a 3x3 rotation matrix
function e_R = exp_rot_gen(x,theta) % Function to create rotation matricies given x axis of rotation and rotation in rad
% Check if only an axis is given. (in this case assume the angle is
% multiplied by a unit vector already ie  x = theta*unit_vec)
if nargin < 2
    theta = 1;
    if norm(x)~=0
        xsk = skew3(x);
        x_mag = norm(x);
        e_R = eye(3) + (sin(x_mag)/x_mag)*xsk + ((1-cos(x_mag))/(x_mag.^2))*(xsk^2);

    else % if the given axis has no magnitude, assume no rotation
        warning('norm of x = 0, returning identity')
        e_R = eye(3);
    end
% If both an axis and angle are given find the rotation matrix
else
    omega = x;
    if norm(omega*theta)~=0
        omegask = skew3(omega);
        e_R = eye(3) + sin(theta)*omegask + ((1-cos(theta))*omegask^2);

    else % if the given axis or angle has no magnitude, assume no rotation
        warning('norm of omega*theta = 0, returning identity')
        e_R = eye(3);
    end
end
end