clc, clear, close all

iiwa = create_iiwa();   % call function

q3_in = (0:2:170)*pi/180;
%SE_delta_RES = zeros(length(q3_in),4);
%poses = zeros(length(q3_in),7);
%pose_index = zeros(length(q3_in),1);

L_in = [0.360 0.420 0.400 0.126]*1000;
dq = 0.01*pi/180;

R_in=rpy2r(0,pi,0, 'xyz'); 
P_in=[0.700,0,0.300];   %

%% find all Poses that has a minimum delta RES with given q3
iter1 = length(q3_in);
for u = 1:iter1
    % calculate inverse kinematic problem
    q_inv = inverse_kinematics(q3_in(u), R_in, P_in, iiwa);% call function
    % deg = q_inv*180/pi
    message1 = ['First for-loop index is: ', int2str(u),'/',int2str(iter1)];
    disp(message1)
    % temp_RES = zeros(8,4);
    iter2 = length(q_inv);
    k = 1;
    n = 1;
    for v = 1:iter2
        message2 = ['Second for-loop index is: ', int2str(v),'/',int2str(iter2)];
        disp(message2)
        q_in = q_inv(:,v)';
        % check q_lim
        q_in_deg = rad2deg(q_in);
        bool(1) = q_in_deg(1) >= -170 && q_in_deg(1) <= 170;
        bool(2) = q_in_deg(2) >= -120 && q_in_deg(2) <= 120;
        bool(3) = q_in_deg(3) >= -170 && q_in_deg(3) <= 170;
        bool(4) = q_in_deg(4) >= -120 && q_in_deg(4) <= 120;
        bool(5) = q_in_deg(5) >= -170 && q_in_deg(5) <= 170;
        bool(6) = q_in_deg(6) >= -120 && q_in_deg(6) <= 120;
        bool(7) = q_in_deg(7) >= -175 && q_in_deg(7) <= 175;
        if bool
            disp('true');
        else
            disp('falsh');
            n = n+1;
            continue;
        end
        % Strukturempfindlichkeit:
        SE = Struktur_Empfindlichkeit(q_in, L_in);
        % Abweichung in einzelne Achse
        deltaSE = SE*dq;
        SE_deltaX = sum(deltaSE(1,:));
        SE_deltaY = sum(deltaSE(2,:));
        SE_deltaZ = sum(deltaSE(3,:));
        % gesamte Fehler
        delta_RES = sqrt(SE_deltaX^2 + SE_deltaY^2 + SE_deltaZ^2);
        temp_config(k,:) = q_in;
        temp_RES(k,1) = SE_deltaX;
        temp_RES(k,2) = SE_deltaY;
        temp_RES(k,3) = SE_deltaZ;
        temp_RES(k,4) = delta_RES;
        k = k+1;
    end
    
    % [SE_min_RES,index] = min(temp_RES(:,4));
    %for m = 1:k
    if n~=iter2
        SE_delta_RES(:,:,u) = temp_RES;
        config(:,:,u) = temp_config;
    end
end

%% Save data into Excel
% xlswrite('Posen und delta RES.xlsx',delta_RES,);
% writematrix(SE_delta_RES,'Posen und delta RES.xlsx', 'sheet', 'delta_RES');
% writematrix(poses,'Posen und delta RES.xlsx', 'sheet', 'Poses_Rad');
% writematrix(poses*180/pi,'Posen und delta RES.xlsx', 'sheet', 'Poses_Degree');
% writematrix(pose_index,'Posen und delta RES.xlsx', 'sheet', 'Pose_index');

%% Read data from Excel
SE_delta_RES = xlsread('Posen und delta RES.xlsx','delta_RES');
poses = xlsread('Posen und delta RES.xlsx','Poses_Rad');
pose_index = xlsread('Posen und delta RES.xlsx','Pose_index');

%% find the best Posen
[SE_min_RES,index] = min(SE_delta_RES(:,4));
best_pose = poses(index,:);
disp('Minimum delta RES(ME):');
disp(SE_min_RES);
disp('The best pose:');
disp(best_pose*180/pi);