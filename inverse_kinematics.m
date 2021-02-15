function [q_out] = inverse_kinematics(q3_in, R_in, P_in, Robot)
% Inverse kinematic calculation
% q3_in: Angle of joint 3, with which to obtain the solution.
% R_in: Orientation of TCP, 3x3
% P_in: Position of TCP, 3x1
% q_inv: return 8 solutions, 7x8

q_out = zeros(7, 8);
DH = [Robot.theta', Robot.d', Robot.a', Robot.alpha'];
% convert homo. trans. to SE3
T0_7_base=SE3.check(rt2tr(R_in, P_in));
% base frame
base_frame = Robot.base;
% wrt. global frame
T0_7 = inv(base_frame)*T0_7_base;

P0_7 = T0_7.t;

P0_6 = real(P0_7 - DH(7, 2)*T0_7.a);    % Wrist

T0_1 = Robot.links(1).A(0);
P0_1 = T0_1.t;

r1_4 = real(norm(P0_1 - P0_6)); % distance from joint 1 to joint 4

q4(1) = real(pi - acos( (DH(3, 2)^2 + DH(5, 2)^2 - r1_4^2) / ...
                    (2*DH(3, 2)*DH(5, 2)) ));

q4(2) = real(pi + acos( (DH(3, 2)^2 + DH(5, 2)^2 - r1_4^2) / ...
                    (2*DH(3, 2)*DH(5, 2)) ));

q3 = q3_in;


q_out(3, :) = q3;
q_out(4, 1:2) = [q4(1), q4(1)];
q_out(4, 3:4) = [q4(2), q4(2)];
q_out(4, 5:8) = q_out(4, 1:4);

d1 = DH(1, 2);
xw = P0_6(1);
yw = P0_6(2);
zw = P0_6(3);

n = 1;
for i = 1:2
    
    T2_5 = Robot.A([3 4 5], [0 0 q3 q4(i) 0 0 0]);
    
    a = real(T2_5.t(3));
    b = real(-T2_5.t(1));
    R_ab = (a^2 + b^2)^0.5;
    alpha_conv = real(atan2(b, a));
    q2(1) = real(alpha_conv - acos((zw - d1)/R_ab));
    q2(2) = real(alpha_conv + acos((zw - d1)/R_ab));
    
    for j = 1:2
        
        T1_5 = Robot.links(2).A(q2(j))*T2_5; % T15 = T12*T25
        
        a = real(T1_5.t(1));
        b = real(-T1_5.t(3));
        R_ab = (a^2 + b^2)^0.5;
        alpha_conv = real(atan2(b, a));
    
        q1(1) = real(alpha_conv - acos(xw/R_ab));
        q1(2) = real(alpha_conv + acos(xw/R_ab));
    
        T0_4_actual_q1_1 = Robot.A([1 2 3 4 5], [q1(1), q2(j), q3, q4(i), 0, 0, 0]);
        T0_4_actual_q1_2 = Robot.A([1 2 3 4 5], [q1(2), q2(j), q3, q4(i), 0, 0, 0]);

        d_q1 = abs(norm(T0_4_actual_q1_1.t - [xw, yw, zw]'));
        d_q2 = abs(norm(T0_4_actual_q1_2.t - [xw, yw, zw]'));
    
        if (d_q1 <= d_q2)
            q1 = q1(1);
        else
            q1 = q1(2);
        end
    
        q_out(1, n) = q1;
        q_out(2, n) = q2(j);
        q_out(1, n + 4) = q1;
        q_out(2, n + 4) = q2(j);
    
        T0_4 = Robot.A([1 2 3 4], ...
            [q_out(1, n), q_out(2, n), q_out(3, n), q_out(4, n), 0 0 0]);
        T4_7 = inv(T0_4) * T0_7;
    
        q6 = -atan2(sqrt(T4_7.n(3)^2 + T4_7.o(3)^2), T4_7.a(3));
        q7 = atan2(-T4_7.o(3), T4_7.n(3));
        q5 = atan2(T4_7.a(2), T4_7.a(1)) + pi;
        q_out(5, n) = q5;
        q_out(6, n) = q6;
        q_out(7, n) = q7;
    
        q6 = atan2(sqrt(T4_7.n(3)^2 + T4_7.o(3)^2), T4_7.a(3));
        q7 = atan2(-T4_7.o(3), T4_7.n(3)) - pi;
        q5 = atan2(T4_7.a(2), T4_7.a(1));
        q_out(5, n + 4) = q5;
        q_out(6, n + 4) = q6;
        q_out(7, n + 4) = q7;

        n = n + 1;
    end
end

% Normalize joint angle in [-pi, pi]
for k = 1:7
    for m = 1:8
        if q_out(k, m) > pi
            q_out(k, m) = q_out(k, m) - 2*pi;
        elseif q_out(k, m) < -pi
            q_out(k, m) = q_out(k, m) + 2*pi;
        end
    end
end

end