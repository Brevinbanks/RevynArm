
clc
% close all
clear arduinoObj
arduinoObj = serialport("COM3",9600)


configureTerminator(arduinoObj,"CR/LF");

flush(arduinoObj)


% Read the ASCII data from the serialport object.
current_t = 0;
k =0;
data1 = readline(arduinoObj);
data2 = readline(arduinoObj);
data3 = readline(arduinoObj);
data = readline(arduinoObj);
fprintf(data1+newline);
fprintf(data2+newline);
fprintf(data3+newline);
fprintf(data+newline);


figure(1)
plot3(0,0,0)
plotf2(eye(4))
xlim([-200,200])
ylim([-200,200])
zlim([-50,550])
title('Base Robot is at home R = eye(4) P = [0,0,486.8750]mm EF:Open')
pause(5)



while(1)



    while strcmp(data,"Enter Desired Joint Angles") ==false
        try
            data = readline(arduinoObj);
        end
    end
    prompt = "Enter angle: ";
    
    if ishandle(1)
        plot3(0,0,0)
    end

    if k==0
        F = [1 0 0 -20;
            0 1 0 0;
            0 0 1 480;
            0 0 0 1];
        ard_angle = IK_Revyn(F,false,false);
        ard_angle = [rad2deg(ard_angle);0]';
        title('Goal 1 R = eye(4) P = [-20,0,480]mm EF:Open')
    end

    if k==1
        F = [1 0 0 0;
            0 1 0 0;
            0 0 1 480;
            0 0 0 1];
        ard_angle = IK_Revyn(F,false,false);
        ard_angle = [rad2deg(ard_angle);2]';
        title('Goal 2 R = eye(4) P = [0,0,480]mm EF:Closed')
    end

    if k==2
        F = [1 0 0 50;
            0 1 0 0;
            0 0 1 480;
            0 0 0 1];
        ard_angle = IK_Revyn(F,false,false);
        ard_angle = [rad2deg(ard_angle);0]';
        title('Goal 3 R = eye(4) P = [50,0,480]mm EF:Open')
    end

    if k==3
        F = [1 0 0 65;
            0 1 0 0;
            0 0 1 475;
            0 0 0 1];
        ard_angle = IK_Revyn(F,false,false);
        ard_angle = [rad2deg(ard_angle);2]';
        title('Goal 4 R = eye(4) P = [65,0,475]mm EF:Closed')
    end

    if k==4
        F=[       0.8160   -0.3272    0.4766  80.0000;
            0.4500    0.8770   -0.1684        25.0000;
            -0.3628    0.3519    0.8628       425.0000;
            0         0         0    1.0000];
        ard_angle = IK_Revyn(F,false,true);
        ard_angle = [rad2deg(ard_angle);0]';
        [X,theta] = extract_axis_angle(F(1:3,1:3));
        X = round(X,2);
        axis_string = "["+string(X(1))+","+string(X(2))+","+string(X(3))+"]";
        theta_string = round(rad2deg(theta),2);

        title("Goal 5 R: axis="+axis_string+" angle="+theta_string+" P = [80.0,25.0,425.0]mm EF:Open")
    end

    


    plotf2(eye(4))
    plotf2(F,'EF')
    plotf2(FK_Revyn(IK_Revyn(F,false,true),5),'J5')
    plotf2(FK_Revyn(IK_Revyn(F,false,true),3),'J3')
    plotf2(FK_Revyn(IK_Revyn(F,false,true),2),'J2')
    hold off
    view(45,45)
    xlim([-200,200])
    ylim([-200,200])
    zlim([-50,550])

    if k>4
        t1 = input(prompt);
    end

    writeline(arduinoObj,string(ard_angle(1)));
    writeline(arduinoObj,string(ard_angle(2)));
    writeline(arduinoObj,string(ard_angle(3)));
    writeline(arduinoObj,string(ard_angle(4)));
    writeline(arduinoObj,string(ard_angle(5)));
    writeline(arduinoObj,string(ard_angle(6)));
    writeline(arduinoObj,string(ard_angle(7)));

    data = readline(arduinoObj);

    while strcmp(data,"Pose Achieved") == false
        try
            data = readline(arduinoObj);
            if strcmp(data," Joint angle out of range")
                break;
            end
        end
    end
    fprintf(data+newline)

    J1_new = ard_angle(1)+90.0;
    J2_new = ard_angle(2)+90.0;
    J3_new = 180.0-ard_angle(3);
    J4_new = ard_angle(4)+90.0;
    J5_new = ard_angle(5)+90;
    J6_new = ard_angle(6)+90;

    J_new =[J1_new,J2_new,J3_new,J4_new,J5_new,J6_new]
    if J1_new>181.0 || J2_new>181.0 || J3_new>181.0 || J4_new>181.0 || J5_new>181.0 || J6_new>181.0
        bad_joints = find(J_new>181)
    end
    if J1_new<-1.0 || J2_new<-1.0|| J3_new<-1.0|| J4_new<-1.0 || J5_new<-1.0 || J6_new<-1.0
        bad_joints = find(J_new<-1.0)
    end
    if J2_new>120.0
        fprintf("Joint 2 angle out of range: "+string(J2_new)+newline);
    end
    if J2_new<60.0
        fprintf("Joint 2 angle out of range: "+string(J2_new)+newline);
    end

    pause(5);
    k=k+1;
end


