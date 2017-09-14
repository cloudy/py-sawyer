%% Sawyer Robot Foward Kinematics
%% by Michail Theofanidis

function [Te,T_f,T] = getSawyerFK_R(q)

%% angle values
theta0=q(1);
theta1=q(2)+deg2rad(90);
theta2=q(3);
theta3=q(4);
theta4=q(5);
theta5=q(6);
theta6=q(7)+deg2rad(170);

%% robot parameters
l_0=0.0794732;

l_1=0.237;
a_1=0.0814619;
d_1=0.0499419;

l_2=0.142537;
d_2=-0.140042;

l_3=0.259989;
d_3=-0.0419592;

l_4=-0.126442;
d_4=-0.1224936;

l_5=0.274653;
d_5=0.031188;

l_6=0.105515;
d_6=-0.109824;

%% frames
T(:,:,1)=[cos(theta0) -sin(theta0) 0 0;
    sin(theta0) cos(theta0) 0 0;
    0 0 1 l_0;
    0 0 0 1;];

T(:,:,2)=[cos(theta1) -sin(theta1) 0 a_1;
    0 0 1 d_1;
    -sin(theta1) -cos(theta1) 0 l_1;
    0 0 0 1;];

T(:,:,3)=[cos(theta2) -sin(theta2) 0 0;
    0 0 -1 d_2;
    sin(theta2) cos(theta2) 0 l_2;
    0 0 0 1;];

T(:,:,4)=[cos(theta3) -sin(theta3) 0 0;
    0 0 1 d_3;
    -sin(theta3) -cos(theta3) 0 l_3;
    0 0 0 1;];

T(:,:,5)=[cos(theta4) -sin(theta4) 0 0;
    0 0 -1 d_4;
    sin(theta4) cos(theta4) 0 l_4;
    0 0 0 1;];

T(:,:,6)=[cos(theta5) -sin(theta5) 0 0;
    0 0 1 d_5;
    -sin(theta5) -cos(theta5) 0 l_5;
    0 0 0 1;];

T(:,:,7)=[cos(theta6) -sin(theta6) 0 0;
    0 0 -1 d_6;
    sin(theta6) cos(theta6) 0 l_6;
    0 0 0 1;];

T(:,:,8)=[0 -1 0 0;
          1 0 0 0;
          0 0 1 0.0245;
          0 0 0 1;];
            
T(:,:,9)=[1 0 0 0;
          0 1 0 0;
          0 0 1 -0.005;
          0 0 0 1;];
       
T(:,:,10)=[1 0 0 0;
           0 1 0 0;
           0 0 1 0.05;
           0 0 0 1;];

%% number of transforms
N=size(T);

%% forward pass of the tree
T_f=T(:,:,1);

for i=2:(N(3))
    
    T_f(:,:,i)=T_f(:,:,i-1)*T(:,:,i);
    
end

%% Final Frame
Te=T_f(:,:,N(3));

end

