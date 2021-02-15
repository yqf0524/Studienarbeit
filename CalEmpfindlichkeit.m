%% create KUKA iiwa
clc, clear, close all

iiwa = create_iiwa();

%% calculate inverse kinematic problem
R_in=rpy2r(0,pi,0, 'xyz'); 
P_in=[0.7,0,.300];

q3 = [0 pi/6 pi/3 pi/2];
q_in_3=q3(1);
q_inv = inverse_kinematics(q_in_3, R_in, P_in, iiwa);
deg = q_inv*180/pi

%% Strukturempfindlichkeit:
L_in = [0.360 0.420 0.400 0.126]*1000;
q_in = q_inv(:,2)';
dq = 0.01*pi/180;
dL=0.02;

SE = Struktur_Empfindlichkeit(q_in, L_in);
ME = Masstoleranz_Empfindlichkeit(q_in, L_in);

% Abweichung in einzelne Achse
deltaSE = SE*dq;
deltaME = ME*dL;

SE_deltaX = sum(deltaSE(1,:));
SE_deltaY = sum(deltaSE(2,:));
SE_deltaZ = sum(deltaSE(3,:));

ME_deltaX = sum(deltaME(1,:));
ME_deltaY = sum(deltaME(2,:));
ME_deltaZ = sum(deltaME(3,:));

% gesamte Fehler
SE_delta_RES = sqrt(SE_deltaX^2 + SE_deltaY^2 + SE_deltaZ^2);
ME_delta_RES = sqrt(ME_deltaX^2 + ME_deltaY^2 + ME_deltaZ^2);