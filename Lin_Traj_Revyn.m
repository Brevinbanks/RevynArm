% Created 8/13/2023 By Brevin Banks
% Modified 8/13/2023 By Brevin Banks
% Trajectory Planning for the Revyn arm using IK
function Jointtraj = Lin_Traj_Revyn(Start_frame, Goal_position,plotit)

if nargin<3
    plotit=false;
end


% Start_frame = [0 0 -1  -300.169610181001;
%     0 -1 0 -25.0859891314089;
%     -1 0  0 220;
%     0 0 0   1];
Start_angles = IK_Revyn(Start_frame,false,true);
Start_position = round(Start_frame(1:3,4),3);

% Goal_position = [-300.537657749172;-2.85625303229396;175];
% Goal_position = [-300.537657749172;25.85625303229396;220];

Goal_frame = [Start_frame(1:3,1:3), Goal_position; 0 0 0 1];
angles = IK_Revyn(Goal_frame,false,true);
Predicted_frame =FK_Revyn(angles,7);

error_r = abs(sum(sum(abs(Goal_frame(1:3,1:3)\Predicted_frame(1:3,1:3))-eye(3))));


error_p = norm(Goal_frame(1:3,4)-Predicted_frame(1:3,4)); % calculate the error between Ik return frame and true frame


fprintf('\n')
Start2Goal=Goal_position-Start_position;
traj_distance = norm(Start2Goal);
num_intervals = traj_distance/2; % Intervals of 2mm

direction2Goal = Start2Goal/norm(Start2Goal);


iterative_add = (direction2Goal*2);

Intermediate_position = Start_position;
for k=2:num_intervals
    k;
    Intermediate_position(:,k) = Intermediate_position(:,k-1) + iterative_add;
    Inter_frame = [Start_frame(1:3,1:3), Intermediate_position(:,k);0 0 0 1];
    Jointtraj(1:6,k-1) = IK_Revyn(Inter_frame,false,false);
    mid_Predicted_frame =FK_Revyn(Jointtraj(1:6,k-1),7);
    error_fm = abs(sum(sum(abs(Goal_frame(1:3,1:3)\mid_Predicted_frame(1:3,1:3))-eye(3))));
end
k = k-1;

Inter_frame = [Start_frame(1:3,1:3), Goal_position;0 0 0 1];
Jointtraj(1:6,k) = IK_Revyn(Inter_frame,false,false);





fprintf('\n Achieving Position with Frame:')

FK_Revyn(Jointtraj(:,k),7)

fprintf("Linear Trajectory Solved with Rotation Error: " + string(error_r) + " and Position Error: " + string(error_p) + newline)

if plotit==true

    if ishandle(1)
        plot3(0,0,0)
    else
    % figure('units','normalized','outerposition',[0 0 1 1])
    end

    for j = 1:2:k
        input_ang = Jointtraj(:,j);
        TF0 = FK_Revyn(input_ang,0);
        TF01 = FK_Revyn(input_ang,1);
        TF02 = FK_Revyn(input_ang,2);
        TF03 = FK_Revyn(input_ang,3);
        TF04 = FK_Revyn(input_ang,4);
        TF05 = FK_Revyn(input_ang,5);
        TF06 = FK_Revyn(input_ang,6);
        TF0ef = FK_Revyn(input_ang,7);
        P01 = TF01(1:3,4);
        P03 = TF03(1:3,4);
        P05 = TF05(1:3,4);
        P0ef = TF0ef(1:3,4);

        TFstart = FK_Revyn(Jointtraj(:,1),7);


        plotv3([0;0;0],P01,'k')
        hold on
        plotv3([0;0;0],P03,'m')
        hold on
        plotv3([0;0;0],P05,'r')
        hold on
        plotv3(P01,P03,'k')
        hold on
        plotv3(P03,P05,'k')
        hold on
        plotv3(P05,P0ef,'k')
        hold on
        plotv3(Start_position,Goal_position,'g')
        plotf2(TF0ef)
        hold on
        plotf2(TFstart)
        hold on
        plotv3(Start_position,TF0ef(1:3,4),'r')

        plot3(Goal_position(1),Goal_position(2),Goal_position(3),'r*')
        for g = 1:j
            TFmid = FK_Revyn(Jointtraj(:,g),7);
            plot3(TFmid(1:3,1),TFmid(1:3,2),TFmid(1:3,3),'.');
        end
        xlim([-400 400])
        ylim([-400 400])
        zlim([-400 400])
        view(80,135)
        pause(0.1)
        hold off
    end

end
end


