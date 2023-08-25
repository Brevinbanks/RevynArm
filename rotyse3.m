% Created 8/15/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function calculates a 4x4 se3 rotation matrix about y given an angle theta
% Input
%   theta - angle in radians of desired rotation
% Output
%   Tf - a 4x4 se3 matrix of a rotation about the y axis
function Tf = rotyse3(theta)
    Tf = [cos(theta) 0 sin(theta) 0;
          0 1 0 0; 
         -sin(theta) 0 cos(theta) 0;
          0 0 0 1];
end