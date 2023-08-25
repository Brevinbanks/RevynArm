% Created 8/15/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function calculates a 3x3 so3 rotation matrix about y given an angle
% theta
% Input
%   theta - angle in radians of desired rotation
% Output
%   ry - a 3x3 so3 matrix of a rotation about the y axis
function ry = roty(theta) %Rotation matrix in y given theta

    ry = [cosd(theta) 0 sind(theta);
        0 1 0; 
        -sind(theta) 0 cosd(theta)];

end
