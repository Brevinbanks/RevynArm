% Created 8/13/2023 By Brevin Banks
% Modified 8/13/2023 By Brevin Banks
% Trajectory Planning for the Revyn arm using IK
clc
clear all




% Start_frame = FK_Revyn([0;0;0;0;0;0],7);


% Start_frame = [-1 0 0 -350;
%                 0 -1 0 -30;
%                 0 0  1  30;
%                 0 0 0   1];

Start_frame = [0 0 -1  -300.169610181001;
                0 -1 0 -50.0859891314089;
                -1 0  0 220;
                0 0 0   1];
Start_angles = IK_Revyn(Start_frame,false,true);
Start_position = round(Start_frame(1:3,4),3);

% Goal_position = [-300.537657749172;-2.85625303229396;175];
Goal_position = [-300.537657749172;50.85625303229396;220];

Goal_frame = [Start_frame(1:3,1:3), Goal_position; 0 0 0 1]
angles = IK_Revyn(Goal_frame,false,true)
Predicted_frame =FK_Revyn(angles,7)
error_f = sum(sum(abs(Goal_frame(1:3,1:3)\Predicted_frame(1:3,1:3)-eye(3))))
splitsolve = false;

error_r = abs(sum(sum(abs(Goal_frame(1:3,1:3)\Predicted_frame(1:3,1:3))-eye(3))));
error_r_180 = abs(sum(sum(abs(Goal_frame(1:3,1:3)\(Predicted_frame(1:3,1:3)*rotz(pi)))-eye(3))));

error_p = norm(Goal_frame(1:3,4)-Predicted_frame(1:3,4)); % calculate the error between Ik return frame and true frame

% if error_p> 2 || error_r>0.1 % 0.1 and 0.01 for loose testing.  2 0.1 for tight testing
%     warning("Final frame orientation cannot be reached"+newline+"Using constrained solver up until orientation cannot be maintained then it will be solved by position only")
%     splitsolve = true;
%     cant_reach = true;
%     while(cant_reach==true)
%         R = rand_rot();
%         RotGoal_frame = [R, Goal_position; 0 0 0 1];
%         RotPredicted_frame =FK_Revyn(IK_Revyn(RotGoal_frame,false,true),7);
%         error_r2 = abs(sum(sum(abs(RotGoal_frame(1:3,1:3)\RotPredicted_frame(1:3,1:3))-eye(3))));
%         fprintf('.')
%         if error_r2>0.1 % 0.1 and 0.01 for loose testing.  2 0.1 for tight testing
%             cant_reach=false;
%         end
%     end
% 
% end
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
    % if error_fm>1
    % 
    %     fprintf("\nk at break: "+string(k))
    %     break;
    % end
end
        k = k-1;

% if splitsolve==true
%     angle_of_rot = acos((trace(R)-1)/2);
%     x = (R(3,2) - R(2,3))/sqrt((R(3,2) - R(2,3))^2+(R(1,3) - R(3,1))^2+(R(2,1) - R(1,2))^2);
%     y = (R(1,3) - R(3,1))/sqrt((R(3,2) - R(2,3))^2+(R(1,3) - R(3,1))^2+(R(2,1) - R(1,2))^2);
%     z = (R(2,1) - R(1,2))/sqrt((R(3,2) - R(2,3))^2+(R(1,3) - R(3,1))^2+(R(2,1) - R(1,2))^2);
%     axis = [x;y;z];
%     mid_theta = 0;
%     angle_step = angle_of_rot/(num_intervals-k);
% 
%     for k=k:num_intervals
%         if k ==1
%             Intermediate_position = Start_position;
%             mid_theta = mid_theta+angle_step;
%             mid_rot = exp_rot_gen(axis,mid_theta);
%             mid_Predicted_frame =FK_Revyn(Start_angles,7);
%             Inter_frame = [mid_Predicted_frame(1:3,1:3)*mid_rot, Intermediate_position(:,k);0 0 0 1];
%             Jointtraj(1:6,k) = IK_Revyn(Inter_frame,false,false);
%         else
%             Intermediate_position(:,k) = Intermediate_position(:,k-1) + iterative_add;
%             mid_theta = mid_theta+angle_step;
%             mid_rot = exp_rot_gen(axis,mid_theta);
%             if k>2
%                 mid_Predicted_frame =FK_Revyn(Jointtraj(1:6,k-2),7);
%             else
%                 mid_Predicted_frame =FK_Revyn(Jointtraj(1:6,k-1),7);
%             end
%             Inter_frame = [mid_Predicted_frame(1:3,1:3)*mid_rot, Intermediate_position(:,k);0 0 0 1];
%             Jointtraj(1:6,k-1) = IK_Revyn(Inter_frame,false,false);
%         end
%         k
% 
%     end
%     Inter_frame = [R, Goal_position;0 0 0 1];
%     Jointtraj(1:6,k+1) = IK_Revyn(Inter_frame,false,false);

% else
    Inter_frame = [Start_frame(1:3,1:3), Goal_position;0 0 0 1];
    Jointtraj(1:6,k) = IK_Revyn(Inter_frame,false,false);
% end




fprintf('\nAchieving Position with orientation:')







if ishandle(1)
    close 1
end
figure(1)

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
    pause(1)
    hold off
end





