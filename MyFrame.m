%% Function that returns the properties of a frame
%% By Michail Theofanidis

function MyFrame(handle,varargin)

%% Options for frame
opt.length = cell2mat((varargin(2)));
opt.color = cell2mat((varargin(4)));
opt.font= cell2mat((varargin(6)));

%% TO DO: Nake Parser!!!

%% Create unit vectors
plot3([0 1]*opt.length,[0 0],[0 0],opt.color,'Parent',handle);
plot3([0 0],[0 1]*opt.length,[0 0],opt.color,'Parent',handle);
plot3([0 0],[0 0],[0 1]*opt.length,opt.color,'Parent',handle);

%% Label the unit vector
text(1*opt.length, 0, 0,'X','Color',opt.color,'FontSize',opt.font,'Parent',handle);
text(0, 1*opt.length, 0,'Y','Color',opt.color,'FontSize',opt.font,'Parent',handle);
text(0, 0, 1*opt.length,'Z','Color',opt.color,'FontSize',opt.font,'Parent',handle);

end

