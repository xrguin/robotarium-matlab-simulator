%% Set up Robotarium object
% Before starting the algorithm, we need to initialize the Robotarium
% object so that we can communicate with the agents

%Initialize Robots FOR MANUAL USE ONLY!
N_actual = 10;
%initialize();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = N_actual;

% Get Robotarium object used to communicate with the robots/simulator
r = Robotarium('NumberOfRobots', N, 'ShowFigure', 'true');

% This is a totally arbitrary number
iterations = 1500; % ~60 sec

%% Experiment constants
% Next, we set up some experiment constants

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

limit = 0.9; % Robotarium boundary limit
avoidanceRadius = r.robot_diameter + 0.01;

% Process the image
im_original= imread('GTLogo.png'); % Original input image file
im_original=flipud(im_original);

% This code ensures that the agents are initially distributed around an
% ellipse.  
xybound = [-0.95, 0.95, -0.95, 0.95];
p_theta = (1:2:2*N)/(2*N)*2*pi;
p_circ = [xybound(2)*cos(p_theta) xybound(2)*cos(p_theta+pi); xybound(4)*sin(p_theta)  xybound(4)*sin(p_theta+pi)];

x_goal = p_circ(:,1:N);


%% Retrieve tools for single-integrator -> unicycle mapping

% Let's retrieve some of the tools we'll need.  We would like a
% single-integrator position controller, a single-integrator barrier
% function, and a mapping from single-integrator to unicycle dynamics

position_int = create_si_position_controller('XVelocityGain', 1, 'YVelocityGain', 1);
%si_barrier_certificate = create_si_barrier_certificate('BarrierGain', 1E6); %%
si_barrier_certificate = create_si_barrier_certificate2('UnsafeBarrierGain', 1e6, 'SafeBarrierGain', 100, 'SafetyRadius', 0.15);
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.75, 'AngularVelocityLimit', pi);

args = {'PositionError', 0.1, 'RotationError', 100};
init_checker = create_is_initialized(args{:});

% Display an image with an associated spatial referencing object.
xImg = linspace(-limit,limit,size(im_original,2));
yImg = linspace(-limit,limit,size(im_original,1));
h = image(xImg,yImg,im_original,'CDataMapping','scaled');

%% Begin the experiment
% This section contains the actual implementation of the barrier
% certificate experiment.

%Iterate for the previously specified number of iterations
for t = 1:iterations
    %tic
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    X = x(1:2,:)';
    
    p_theta = (1:2:2*N)/(2*N)*2*pi + 0.05*0.033*t;
    p_circ = [xybound(2)*cos(p_theta) xybound(2)*cos(p_theta+pi); xybound(4)*sin(p_theta)  xybound(4)*sin(p_theta+pi)];

    x_goal = p_circ(:,1:N);

    dx = position_int(x(1:2, :), x_goal);

    %%
    % Normalization of controls.  This code ensures that
    %%
    % $$
    %   \|dxu\| \leq dmax
    % $$
    dxmax = 0.2;
    for i = 1:N
        if norm(dx(:,i)) > dxmax
            dx(:,i) = dx(:,i)/norm(dx(:,i))*dxmax;
        end
    end

    %% Apply barrier certs. and map to unicycle dynamics

    %Ensure the robots don't collide
    dx = si_barrier_certificate(dx, x);

    % Transform the single-integrator dynamics to unicycle dynamics using a
    % diffeomorphism, which can be found in the utilities
    dx = si_to_uni_dyn(dx, x);

    %% Set the velocities of the agents

    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dx);

    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
    %toc
end   
%
%pause(5);
%delete tag_ids.mat;
%clear all;
%close all;
%charge_robots(1);
