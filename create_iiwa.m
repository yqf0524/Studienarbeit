function iiwa = create_iiwa()

DH = [0 .360 0 -pi/2 0
      0    0 0  pi/2 0
      0 .420 0 -pi/2 0
      0    0 0  pi/2 0
      0 .400 0 -pi/2 0
      0    0 0  pi/2 0
      0 .126 0   0   0];

%           theta  d  a  alpha offset
% L(1) = Link([0 .360 0 -pi/2 0],'standard');
% L(2) = Link([0 0 0 pi/2 0],'standard');
% L(3) = Link([0 .420 -pi/2 0 0],'standard');
% L(4) = Link([0 0 0 pi/2 0],'standard');
% L(5) = Link([0 .400 0 -pi/2 0],'standard');
% L(6) = Link([0 0 0 pi/2 0],'standard');
% L(7) = Link([0 .126 0 0 0],'standard');

iiwa = SerialLink(DH, 'name', 'KUKA iiwa 14 R820');

iiwa.qlim = deg2rad([-170, 170
                   -120, 120
                   -170, 170
                   -120, 120
                   -170, 170
                   -120, 120
                   -175, 175]);

iiwa.display()

% q_initial = [0, 0, 0, 0, 0, 0, 0];
q_initial = [-120, -60, 78, 62, -83, -100, 148];
workspace = [-2, 2, -2, 2, -2, 3];

view(3);
figure(1)
iiwa.plot(q_initial, 'workspace', workspace, 'scale', 0.5, 'tilesize', 1)
teach(iiwa)

end

