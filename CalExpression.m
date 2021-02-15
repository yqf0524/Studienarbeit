function [dx,dy,dz] = CalExpression(var)

syms q1 q2 q3 q4 q5 q6 q7 d1 d2 d3 d4
q_in = [q1 q2 q3 q4 q5 q6 q7];
L_in = [d1 d2 d3 d4];

Tr = forward_kinematics(q_in, L_in); % call function
 
px = Tr(1,4);
py = Tr(2,4);
pz = Tr(3,4);

dx = diff(px, var);
dy = diff(py, var);
dz = diff(pz, var);

end