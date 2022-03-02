clear;
clc;
close all;

addpath(fullfile('..', 'src'));

%% TODO 3.2: This file should produce all the plots for the deliverable
Ts       = 1/20; % Sample time
rocket   = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

% Design MPC controller for sys_x
Hx = 3;        % Horizon length in seconds
Nx= ceil(Hx/Ts);  % Horizon length in discret steps
mpc_x = MPC_Control_x(sys_x, Ts, Hx);

% Design MPC controller for sys_y
Hy = 3;        % Horizon length in seconds
Ny= ceil(Hy/Ts);  % Horizon length in discret steps
mpc_y = MPC_Control_y(sys_y, Ts, Hy);

% Design MPC controller for sys_z
Hz = 3;        % Horizon length in seconds
Nz= ceil(Hz/Ts);  % Horizon length in discret steps
mpc_z = MPC_Control_z(sys_z, Ts, Hz);

% Design MPC controller for sys_roll
Hroll = 3;        % Horizon length in seconds
Nroll= ceil(Hroll/Ts);  % Horizon length in discret steps
mpc_roll = MPC_Control_roll(sys_roll, Ts, Hroll);


%% TODO: This file should produce all the plots for the deliverable

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);
% Setup reference function
Tf = 30;
ref = @(t_, x_) rocket.MPC_ref(t_, Tf);
x0 = zeros(12,1);
[T, X, U, Ref] = rocket.simulate_f(x0, Tf, mpc, ref);
% Plot pose
rocket.anim_rate = 1; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = 'Merged lin. MPC in nonlinear simulation'; % Set a figure title
