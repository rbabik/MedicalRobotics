function a = calcEndEffectorPos(a, b, g, d)

% Plots in 3D space the robots positions for given joint angles
offsetZ = 20;
l1 = 0;
l2 = 2.5; 
l3 = 8;
l4 = 4;
l5 = 2;

%homogeneous transforms 
%rotation about x by 180 to align spatial with inverted base
H01 = [1 0 0 0; 
    0 -1 0 0;
    0 0 -1 offsetZ - l1;
    0 0 0 1];

% H12 = [cos(a) 0 sin(a) 0; 
%     sin(a) 0 -cos(a) 0;
%     0 1 0 l2;
%     0 0 0 1];
H12 = [0 cos(a) -sin(a) 0; 
    0 sin(a) cos(a) 0;
    1 0 0 l2;
    0 0 0 1];

H23 = [cos(b) -sin(b) 0 l3; 
    sin(b) cos(b) 0 0;
    0 0 1 0;
    0 0 0 1];

H34 = [cos(g) -sin(g) 0 l4; 
    sin(g) cos(g) 0 0;
    0 0 1 0;
    0 0 0 1];
% H34 = [0 -sin(g) cos(g) 0; 
%      0 cos(b) sin(b) 0;
%     -1 0 0 l4;
%     0 0 0 1];

H01*H12*H23*H34;

%construct link vectors
%origin
% F_0 = [0 0 0];
% %link 1
% F_1 = [0 0 l1];
% %link 2
% F_2 = H01 * [0; 0; l2; 1];
% %link 3
% F_3 = H01 * H12 *[l3*cos(b); l3*sin(b); 0; 1];
% %link 4
% F_4 = H01 * H12 * H23 * [l4*cos(g); l4*sin(g); 0; 1];

F_0 = [0 0 20];
%joint 0 location
F_1 = F_0 - [0 0 l1];
%joint 1 location
F_2 = H01 * [0; 0; l2; 1];
%joint 2 location
F_3 = H01 * H12 *[l3*cos(b); l3*sin(b); 0; 1];
%joint 3 location
F_4 = H01 * H12 * H23 * [l4*cos(g); l4*sin(g); 0; 1];
%end effector location
F_5 = H01 * H12 * H23 * H34 * [l5*cos(d); l5*sin(d); 0; 1];

a = F_5(1:3)';
end
