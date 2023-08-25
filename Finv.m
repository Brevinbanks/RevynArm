% Created 8/13/2023 By Brevin Banks
% Modified 8/18/2023 By Brevin Banks
% This function takes and inverts a given homogeneous se3 matrix
% input
%   se3_mat-take in a 4x4 homogeneous transformation matrix 
% output
%   inv_mat - the inverted form of the given se3 matrix
function inv_mat = Finv(se3_mat)
    inv_mat = zeros(4,4);     
    % extract the R and p values in he se3 matrix
    R = se3_mat(1:3,1:3);              
    p = se3_mat(1:3,4); 
    % find transpose of R, transpose is equal to inverted 3x3 rotation
    % matrix
    R_t = transpose(R);                
    p_inv = -R_t*p; 
    % Place inverted Rotation and position elements in the matrix
    inv_mat = [R_t p_inv; zeros(1,3) 1]; 
end 