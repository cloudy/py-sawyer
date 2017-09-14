%% Sawyer Robot Foward Kinematics with DH parameters
%% by Michail Theofanidis

function [Te,T_f,T]=getSawyerFK_DH(q,offset)

%% Get Size of the joint values
len=size(q);

%% angle values

if offset==1
    q(2)=q(2)+deg2rad(90);
    q(7)=q(7)+deg2rad(170)+deg2rad(90);
elseif offset==0
    q(2)=q(2);
    q(7)=q(7);    
elseif offset==2
    q(2)=q(2)-deg2rad(90);
    q(7)=q(7)-deg2rad(170)-deg2rad(90);  
end

%% robot parameters
l_0=0.0794732;

l_1=0.237;
a_1=0.0814619;
d_1=0.0499419;

l_2=0.142537;
d_2=0.140042;

l_3=0.259989;
d_3=0.0419592;

l_4=0.126442;
d_4=0.1224936;

l_5=0.274653;
d_5=0.031188;

l_6=0.105515;
d_6=0.109824;

l_e=0.0695;
%l_e=0.0245;

%% Sawyer DH Table
DH_Table=[a_1 -pi/2 0 q(1);
    0 pi/2 d_1+l_2 q(2);
    0 -pi/2 d_2+l_3 q(3);
    0 pi/2 -(d_3+l_4) q(4);
    0 -pi/2 d_4+l_5 q(5);
    0 pi/2 d_5+l_6 q(6);
    0 0 d_6+l_e q(7);];

%% Initialize the transformations
T=zeros(4,4,len(2));

%% Create the Homogeneous matrixes
for i=1:(len(2))
    
    if i==(len(2))
        
        % This is for the end effector
        T(:,:,i)=DH_matrix(DH_Table(i,1),DH_Table(i,2),DH_Table(i,3),DH_Table(i,4));
        
    else
        
        % This is for the res of the body
        T(:,:,i)=DH_matrix(DH_Table(i,1),DH_Table(i,2),DH_Table(i,3),DH_Table(i,4));
        
    end
    
end

%% number of transforms
N=size(T);

%% forward pass of the tree
T_f=T(:,:,1);

for i=2:(N(3))
    
    T_f(:,:,i)=T_f(:,:,i-1)*T(:,:,i);
    
end

%% add the missing length from the bottom
T_f(3,4,:)=T_f(3,4,:)+l_0+l_1;

%% Final Frame
Te=T_f(:,:,N(3));

end
