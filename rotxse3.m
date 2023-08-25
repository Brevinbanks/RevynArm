% Created 8/15/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function calculates a 4x4 se3 rotation matrix about x given an angle phi
% Input
%   phi - angle in radians of desired rotation
% Output
%   Tf - a 4x4 se3 matrix of a rotation about the x axis
function Tf = rotxse3(phi)
    Tf = [1 0 0 0; 
          0 cos(phi) -sin(phi) 0; 
          0 sin(phi) cos(phi)  0;
          0 0 0 1];
end