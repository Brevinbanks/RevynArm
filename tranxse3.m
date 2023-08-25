% Created 8/15/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function calculates a 4x4 se3 translation transformation matrix 
% along x given a distance alpha
% Input
%   alpha - desired distance to translate in x
% Output
%   Tf - a 4x4 se3 matrix of a translation
function Tf = tranxse3(alpha)
    Tf = [1 0 0 alpha;
          0 1 0 0; 
         0 0 1  0;
          0 0 0 1];
end