% Created 8/15/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function calculates a 3x3 so3 rotation matrix about x given an angle
% phi
% Input
%   phi - angle in radians of desired rotation
% Output
%   rx - a 3x3 so3 matrix of a rotation about the x axis
function rx = rotx(phi) %Rotation matrix in x given phi
 
        rx = [1 0 0;
              0 cos(phi) -sin(phi);
              0 sin(phi) cos(phi)   ];

end
