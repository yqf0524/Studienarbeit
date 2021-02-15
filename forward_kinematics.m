% forward kinematics using DH parameter
% DH(theta,d,a,alpha)

function Tr = forward_kinematics(q_in, L_in)

DH = [q_in(1) L_in(1) 0 -pi/2 0
      q_in(2)   0     0  pi/2 0
      q_in(3) L_in(2) 0 -pi/2 0
      q_in(4)   0     0  pi/2 0
      q_in(5) L_in(3) 0 -pi/2 0
      q_in(6)   0     0  pi/2 0
      q_in(7) L_in(4) 0   0   0];

    first=1;
    last=length(q_in);
    
 for i=first:last
    q=DH(i,1);
    d=DH(i,2);
    a=DH(i,3);
    al=DH(i,4); 
    T(:,:,i)=[cos(q)     -sin(q)*cos(al)      sin(q)*sin(al)    a*cos(q);
              sin(q)      cos(q)*cos(al)     -cos(q)*sin(al)    a*sin(q);
                0           sin(al)            cos(al)             d;
                0               0                       0           1];
 
 end
 Tr=eye(4);
for i=first:last
    Tr=Tr*T(:,:,i);
end
end

