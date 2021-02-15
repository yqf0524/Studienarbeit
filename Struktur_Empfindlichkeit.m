% q_in: joint angle of robotic

function SE = Struktur_Empfindlichkeit(q_in, L_in)

% Strukturempfindlichkeit:
SEdx = zeros(7,1);
SEdy = zeros(7,1);
SEdz = zeros(7,1);

syms q1 q2 q3 q4 q5 q6 q7 d1 d2 d3 d4
q_var = [q1 q2 q3 q4 q5 q6 q7];
L_var = [d1 d2 d3 d4];

for idx = 1 : length(q_var)
    
    [dx,dy,dz] = CalExpression(q_var(idx)); % function
    
    SEdx(idx) = subs(dx, [q_var L_var], [q_in L_in]);
    SEdy(idx) = subs(dy, [q_var L_var], [q_in L_in]);
    SEdz(idx) = subs(dz, [q_var L_var], [q_in L_in]);
   
end

SE = [SEdx SEdy SEdz]';

end