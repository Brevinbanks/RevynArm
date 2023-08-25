% Created 8/15/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function calculates a 4x4 se3 translation transformation matrix 
% along z given a distance gamma
% Input
%   gamma - desired distance to translate in z
% Output
%   Tf - a 4x4 se3 matrix of a translation
function Tf = tranzse3(gamma)
    Tf = [1 0 0 0;
          0 1 0 0; 
         0 0 1  gamma;
          0 0 0 1];
end