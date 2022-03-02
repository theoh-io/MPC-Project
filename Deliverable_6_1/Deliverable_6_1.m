clc ;
clear;
addpath(fullfile('..', 'src'));

Ts = 1/10; % Note that we choose a larger Ts here to speed up the simulation
rocket = Rocket(Ts);
H = 2;
nmpc = NMPC_Control(rocket, H);

% MPC reference with default maximum roll = 15 deg
Tf = 30;
ref = @(t , x ) rocket.MPC_ref(t , Tf, deg2rad(50));
x0=zeros(12,1);

[T, X, U, Ref] = rocket.simulate_f(x0, Tf, nmpc, ref);

% Plot pose
rocket.anim_rate = 1; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title

%% TODO: This file should produce all the plots for the deliverable