% Revyn IK Unit Testing Script
% The following script contains 2 sets of tests. The first set is an
% iterative unit test for the revyn arm IK_Revyn inverse kinematics
% solutions. It runs 10000 randomly generated Ik tests over the reachable
% workspace.
% The second set contains commented out sections of input angles that
% represent end cases and potentially problematic cases. Some cases are not
% truly reachable by the robot, and some of these bad cases should fail.
% That is intended.

d1f = 88.95; %mm
for test_num = 1:10000 % Run 10000 IK tests
    % Generate junk frame to start while loop conditions
    Test_frame = eye(4);
    Test_frame(3,4) = -1;
    out_of_reach = true;
    while (out_of_reach==true)
        input_ang = (-pi + (pi+pi) .* rand(6,1)); % Generate 6 joint angles
        input_ang(3)=(0 + (pi) .* rand(1,1)); % regenerate 3rd joint angle in proper range
        
        Test_frame = Fk_Revyn(input_ang,7); % Find the frame at the end effector
        
        % Check if generated frame is reachable by the robot
        if Test_frame(3,4)<0 
            out_of_reach=true;
        elseif norm(Test_frame(1:3,4))<202.635
            out_of_reach=true;
        elseif norm(Test_frame(1:3,4)-[0;0;d1f])<205.5
            out_of_reach=true;
        else
            out_of_reach=false;
        end
    end
tic % Start test timer
return_ang = IK_Revyn(Test_frame,false,true); % Run IK test
time_passed = toc; % record time
result_frame = Fk_Revyn(return_ang,7); % Capture the resultant end effector frame given IK joints
pass_or_fail = 1;
error = sum(sum(abs((Test_frame*(result_frame)^-1)-eye(4)))); % calculate the error between Ik return frame and true frame
if error> 5 % 5 for loose testing.  0.1 for tight testing
% Display failed test data
    Test_frame
    result_frame
    er_str = string(error);
    warning("Unit Test Failure "+er_str+" error")
    pass_or_fail = 0;
    input_ang
    return_ang
    J = Jac_Revyn(input_ang);
    Determinant = det(J)
    if ishandle(1)
        close 1
    end
    figure(1)
    subplot(1,2,1)
    plotf2(Fk_Revyn(input_ang,0),'Base')
    hold on
    plotf2(Fk_Revyn(input_ang,1),'1')
    hold on
    plotf2dc(Fk_Revyn(input_ang,2),'2')
    hold on
    plotf2(Fk_Revyn(input_ang,3),'3')
    hold on
    plotf2dc(Fk_Revyn(input_ang,4),'4')
    hold on
    plotf2(Fk_Revyn(input_ang,5),'5')
    hold on
    plotf2dc(Fk_Revyn(input_ang,6),'6')
    hold on
    plotf2(Fk_Revyn(input_ang,7),'EF')
    hold on
    xlim([-400 400])
    ylim([-400 400])
    zlim([-400 400])
    view(25,35)

    subplot(1,2,2)
    plotf2(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7),false,true),0),'Base')
    hold on
    plotf2(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7),false,true),1),'1')
    hold on
    plotf2dc(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7),false,true),2),'2')
    hold on
    plotf2(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7),false,true),3),'3')
    hold on
    plotf2dc(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7),false,true),4),'4')
    hold on
    plotf2(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7),false,true),5),'5')
    hold on
    plotf2dc(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7),false,true),6),'6')
    hold on
    plotf2(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7),false,true),7),'EF')
    hold on
    xlim([-400 400])
    ylim([-400 400])
    zlim([-400 400])
    view(25,35)
    IK_Revyn(Fk_Revyn(input_ang,7),true,true);
else % if test passed display passed with the recorded error
    er_str = string(error);
    fprintf("Unit Test Passed with "+er_str+" error\n")

end
Test_details(test_num,1:15) = [time_passed,pass_or_fail,error,input_ang',Test_frame(1:3,4)',result_frame(1:3,4)'];
end
% Produce test statitistics
Passed_tests = find(Test_details(:,2)==1);
Failed_tests = find(Test_details(:,2)==0);
Passed_test_details = Test_details(Passed_tests,:);
Failed_test_details = Test_details(Failed_tests,:);
average_time = mean(Passed_test_details(:,1))
average_error = mean(Passed_test_details(:,3))
num_fails = sum(Test_details(:,2)==0)

if ishandle(3)
    close 3
end
figure(3)
axis equal
hold on
plot3(Test_details(:,10),Test_details(:,11),Test_details(:,12),'k.')
plot3(Passed_test_details(:,13),Passed_test_details(:,14),Passed_test_details(:,15),'b.')
plot3(Failed_test_details(:,13),Failed_test_details(:,14),Failed_test_details(:,15),'b.')


%%
if ishandle(1)
    close 1
end
% input_ang = [-2.72869852930877	0.500530799590759	0.887190784110116	-2.29941192386606	-1.90585486555730	0.289566914803744]';
% input_ang = [0.683188038437189	0.197044872607062	1.29978714042636	-2.67209541000576	1.40067271842472	2.76429052646683]';
% input_ang = [0.959713329567054	1.96803585159191	-0.754375359710328	-0.933036395907252	-1.94163863059122	-2.37172069636491]';
% input_ang = [0.517269242893840	0.555243960990711	0.814947226781028	0.657913312429094	1.31050697859433	1.39131398050429]';
% input_ang = [0.1054; 0.8806; 2.3182; 0.6291; 3.1395;-2.3300];
% input_ang = [ 0.0081;0.4290;3.1263;1.9579;0.0902;2.4784];
% input_ang = [-0.6354;-0.5335;-2.0060;-1.5370;-3.0126; 2.6620]
% input_ang=[0.1739;-0.1287; 1.8934;-1.7100;-0.0120; 2.5186];
% input_ang = [0.2810;0.9256;0.2757;1.3889;0.1413;3.1020];
% input_ang = [-3.0750;-1.8642; 1.5708; 3.1416;-1.4631;3.1384];
% input_ang = [0.0997;-1.9952; 1.5708; 3.1416;-1.9373;-0.0299];
% input_ang = [0.0997;-1.9952; 1.5708; 0.0000; 1.9373; 0.0997];
% input_ang = [0;-0.8017;-0.3234; 0.0000; 2.6959;-0.0000];
% input_ang = [0;-2.7480; 1.5708; 0.0000; 2.5582;-0.0000];
% input_ang = [0;pi/8;-pi/4;0;0;0];
% input_ang = [pi/3;pi/8;-pi/4;-pi/6;0;0];
% input_ang = [-pi/2;0;pi/5;-3*pi/4;-pi/3;pi/5];
% input_ang = [0;pi/4;0;0;0;0];
% input_ang = [0;-pi/2.5;pi/1.3;pi/4;pi/2;0];
% input_ang = [0;0;0;pi/4;pi/2;0];
% input_ang = [0;0;pi/2;0;0;0]
% input_ang = [0;0;0;0;0;0];
J = Jac_Revyn(input_ang)
det(J)
figure(1)
subplot(1,2,1)
plotf2(Fk_Revyn(input_ang,0),'Base')
hold on
plotf2(Fk_Revyn(input_ang,1),'1')
hold on
plotf2dc(Fk_Revyn(input_ang,2),'2')
hold on
plotf2(Fk_Revyn(input_ang,3),'3')
hold on
plotf2dc(Fk_Revyn(input_ang,4),'4')
hold on
plotf2(Fk_Revyn(input_ang,5),'5')
hold on
plotf2dc(Fk_Revyn(input_ang,6),'6')
hold on
plotf2(Fk_Revyn(input_ang,7),'EF')
hold on
xlim([-400 400])
ylim([-400 400])
zlim([-400 400])
view(25,35)

subplot(1,2,2)
plotf2(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7)),0),'Base')
hold on
plotf2(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7)),1),'1')
hold on
plotf2dc(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7)),2),'2')
hold on
plotf2(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7)),3),'3')
hold on
plotf2dc(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7)),4),'4')
hold on
plotf2(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7)),5),'5')
hold on
plotf2dc(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7)),6),'6')
hold on
plotf2(Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7)),7),'EF')
hold on
xlim([-400 400])
ylim([-400 400])
zlim([-400 400])
view(25,35)
Fk_Revyn(input_ang,7)
Fk_Revyn(IK_Revyn(Fk_Revyn(input_ang,7)),7)
IK_Revyn(Fk_Revyn(input_ang,7),true)


