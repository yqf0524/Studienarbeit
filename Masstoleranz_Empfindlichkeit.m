% L_in: length of robotic links

function ME = Masstoleranz_Empfindlichkeit(q_in, L_in)

% Maﬂtoleranzempfindlichkeit:
MEdx = zeros(4,1);
MEdy = zeros(4,1);
MEdz = zeros(4,1);

syms q1 q2 q3 q4 q5 q6 q7 d1 d2 d3 d4
q_var = [q1 q2 q3 q4 q5 q6 q7];
L_var = [d1 d2 d3 d4];

for idx = 1 : length(L_var)
    [dx,dy,dz] = CalExpression(L_var(idx));    % call function
    
    MEdx(idx) = subs(dx, [q_var L_var], [q_in L_in]);
    MEdy(idx) = subs(dy, [q_var L_var], [q_in L_in]);
    MEdz(idx) = subs(dz, [q_var L_var], [q_in L_in]);
end

ME = [MEdx MEdy MEdz]';

end