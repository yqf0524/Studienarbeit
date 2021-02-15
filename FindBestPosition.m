clc, clear, close all

iiwa = create_iiwa();   % call function

q3_in = 0;
R_in = rpy2r(0,pi,0, 'xyz');
phi = 0:2:360;
P_in = zeros(length(phi),3);
L_in = [0.360 0.420 0.400 0.126]*1000;
dL=0.02;

ME_delta_RES = zeros(length(P_in),4);

for k = 1:length(phi)
   px = 0.700 + 0.02*cosd(phi(k));
   py = 0.02*sind(phi(k)); 
   pz = 0.300;
   P_in(k,:) = [px,py,pz];
end

%% find all Positions that have a minimum delta RES with given Position P_in
for idx = 1:length(phi)
    message = ['For-loop phi=', int2str(phi(idx))];
    disp(message)
    % calculate inverse kinematic problem
    q_inv = inverse_kinematics(q3_in, R_in, P_in(idx,:), iiwa); % call function
    q_in = q_inv(:,6)';
    % Masstoleranzempfindlichkeit:
    ME = Masstoleranz_Empfindlichkeit(q_in, L_in);
    % Abweichung in einzelne Achse
    deltaME = ME*dL;
    ME_deltaX = sum(deltaME(1,:));
    ME_deltaY = sum(deltaME(2,:));
    ME_deltaZ = sum(deltaME(3,:));
    % gesamte Fehler
    delta_RES = sqrt(ME_deltaX^2 + ME_deltaY^2 + ME_deltaZ^2);
    ME_delta_RES(idx,1) = ME_deltaX;
    ME_delta_RES(idx,2) = ME_deltaY;
    ME_delta_RES(idx,3) = ME_deltaZ;
    ME_delta_RES(idx,4) = delta_RES;
end

%% Save data into Excel
writematrix(P_in,'Position und delta RES.xlsx', 'sheet', 'P_in');
writematrix(ME_delta_RES,'Position und delta RES.xlsx', 'sheet', 'ME_delta_RES');

%% find the best Position and Pose
[ME_min_RES,index] = min(ME_delta_RES(:,4));
best_position = P_in(index,:);
disp('Minimum delta RES(SE):');
disp(ME_min_RES);
disp('The best positon: ');
disp(best_position);

%% find best Pose at the best_Position
q3_in = (0:2:170)*pi/180;
SE_delta_RES = zeros(length(q3_in),4);
poses = zeros(length(q3_in),7);
R_in=rpy2r(0,pi,0, 'xyz');
P_in_new = best_position;
dq = 0.01*pi/180;

%% find all Poses that has a minimum delta RES with given q3
iter1 = length(q3_in);
for u = 1:iter1
    % calculate inverse kinematic problem
    q_inv = inverse_kinematics(q3_in(u), R_in, P_in_new, iiwa);% call function
    % deg = q_inv*180/pi
    message1 = ['First for-loop index is: ', int2str(u),'/',int2str(iter1)];
    disp(message1)
    temp_RES = zeros(8,4);
    iter2 = length(q_inv);
    for v = 1:iter2
        message2 = ['Second for-loop index is: ', int2str(v),'/',int2str(iter2)];
        disp(message2)
        q_in = q_inv(:,v)';
        % Strukturempfindlichkeit:
        SE = Struktur_Empfindlichkeit(q_in, L_in);
        % Abweichung in einzelne Achse
        deltaSE = SE*dq;
        SE_deltaX = sum(deltaSE(1,:));
        SE_deltaY = sum(deltaSE(2,:));
        SE_deltaZ = sum(deltaSE(3,:));
        % gesamte Fehler
        delta_RES = sqrt(SE_deltaX^2 + SE_deltaY^2 + SE_deltaZ^2);
        temp_RES(v,1) = SE_deltaX;
        temp_RES(v,2) = SE_deltaY;
        temp_RES(v,3) = SE_deltaZ;
        temp_RES(v,4) = delta_RES;
    end
    
    [min_RES,index] = min(temp_RES(:,4));
    SE_delta_RES(u,1) = temp_RES(index,1);
    SE_delta_RES(u,2) = temp_RES(index,2);
    SE_delta_RES(u,3) = temp_RES(index,3);
    SE_delta_RES(u,4) = min_RES;
    poses(u,:) = q_inv(:,index);
end

%% Save data into Excel
% xlswrite('Posen und delta RES.xlsx',delta_RES,);
% writematrix(SE_delta_RES,'Position und delta RES.xlsx', 'sheet', 'SE_delta_RES');
% writematrix(poses,'Position und delta RES.xlsx', 'sheet', 'Poses_Rad');
% writematrix(poses*180/pi,'Position und delta RES.xlsx', 'sheet', 'Poses_Degree');

%% find the best Posen
[SE_min_RES,index] = min(SE_delta_RES(:,4));
best_pose = poses(index,:);
disp('Minimum delta RES(ME):');
disp(SE_min_RES);
disp('The best pose:');
disp(best_pose*180/pi);



%% =====================================================================
% Read data from Excel
P_in = xlsread('Position und delta RES.xlsx','P_in');
ME_delta_RES = xlsread('Position und delta RES.xlsx','ME_delta_RES');
SE_delta_RES = xlsread('Position und delta RES.xlsx','SE_delta_RES');
poses = xlsread('Position und delta RES.xlsx','Poses_Rad');

%% find the best Position and pose
% best Position
[ME_min_RES,index] = min(ME_delta_RES(:,4));
best_position = P_in(index,:);
% best Pose
[SE_min_RES,index] = min(SE_delta_RES(:,4));
best_pose = poses(index,:);

%% show the results
disp('Minimum delta RES(SE):');
disp(ME_min_RES);
disp('The best positon: ');
disp(best_position);
disp('Minimum delta RES(ME):');
disp(SE_min_RES);
disp('The best pose:');
disp(best_pose*180/pi);