%% Sawyer Robot plot function
%% by Michail Theofanidi

function plotSawyer(T)

%% number of transforms
N=size(T);

%% Frames Options

len=[0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3 0.3];
colors=['k' 'y' 'm' 'c' 'r' 'g' 'b' 'k' 'k' 'k'];
font=5;

%% Define the world frame

% World Options
world.limit = 1.5;
world.axis=['X' 'Y' 'Z'];

% Axis limits
limits = [-world.limit world.limit -world.limit world.limit -world.limit world.limit];
axis(limits);

% Axis labels
xlabel(world.axis(1))
ylabel(world.axis(2))
zlabel(world.axis(3))

% Graph options
cla
grid on
hold on

% World frame
world = gca;

%% Assign the frames to the equivalent joints

% Array to store the frame objects
h=zeros(1,N(3));

for i=1:N(3)
    
    % In the first iteration the frame is the world
    if i==1
        frame=world;
    else
        frame=h(i-1);
    end
    
    % Frame transformation
    h(i) = hgtransform('Parent', frame);
    
    % Draw the Frame
    MyFrame(h(i),'length',len(i),'color',colors(i),'font',font);
    
    % Set the frame Parent relationship
    set(h(i),'Matrix', T(:,:,i));
    
end


%% Attach the links to the world frame

% Initialize the Matrix Containing the position of each Joint
xx=zeros(N(3),1);
yy=zeros(N(3),1);
zz=zeros(N(3),1);

for i=1:N(3)
    
    % get the position of the joint
    if i==1
        Tran=T(:,:,i);
        [x,y,z]=MyTransl(Tran);
    else
        Tran=Tran*T(:,:,i);
        [x,y,z]=MyTransl(Tran);
    end
    
    xx(i+1,1)=x;
    yy(i+1,1)=y;
    zz(i+1,1)=z;
    
end

% Set the links Parent relationship
plot3(xx,yy,zz,'ko-','Linewidth',1.5,'Parent',world)

% Plot now
drawnow

end

