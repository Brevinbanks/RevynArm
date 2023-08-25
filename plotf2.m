% Created 8/13/2023 By Brevin Banks
% Modified 8/20/2023 By Brevin Banks
% This function draws a frame using x,y, and z axis unit vectors at the
% given position in the orientation defined by an se3 rotation matrix
% Input
%   m - a 4x4 se3 matrix
%   numorlab - a string for a label next to the axis names
% Output
%   NONE - check the live figure
function plotf2(m,numorlab)

% Check if there is a desired label assigned to accompany the plotted frame
if nargin~=2
    numorlab = '';
end
numorlab = string(numorlab);

[rows, cols] = size(m);
if ((rows ~= 4) || (cols ~= 4))
    error("A matrix with shape ("+string(rows)+","+string(cols)+") was given to plotf2. A 4x4 matrix is needed.");
end

d = det(m(1:3,1:3));
if (abs(d-1)>0.01)
    failstring1 = string(m(1,1))+" "+string(m(1,2))+" "+string(m(1,3))+newline;
    failstring2 = string(m(2,1))+" "+string(m(2,2))+" "+string(m(2,3))+newline;
    failstring3 = string(m(3,1))+" "+string(m(3,2))+" "+string(m(3,3))+newline;
    warning("Given Rotation Matrix: ")
    fprintf(failstring1);
    fprintf(failstring2);
    fprintf(failstring3);
    error('An improper rotation matrix was given to plotf2');
end

% isolate the rotation matrix
zer=m(1:3,4);

hold on

% Plot each axis with a 40 length from the origin x in red, y in green, z
% in cyan
plotv3(zer,zer+40*m(1:3,1),'r',2);
plotv3(zer,zer+40*m(1:3,2),'g',2);
plotv3(zer,zer+40*m(1:3,3),'c',2);
% Draw text next to the axis and a label if given
text(m(1,4)+m(1,1)*30,m(2,4)+m(2,1)*30,m(3,4)+m(3,1)*30,"x"+numorlab);
text(m(1,4)+m(1,2)*30,m(2,4)+m(2,2)*30,m(3,4)+m(3,2)*30,"y"+numorlab);
text(m(1,4)+m(1,3)*30,m(2,4)+m(2,3)*30,m(3,4)+m(3,3)*30,"z"+numorlab);

xlabel('X');
ylabel('Y');
zlabel('Z');

axis equal;
grid on;
