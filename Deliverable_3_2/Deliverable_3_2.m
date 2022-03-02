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


%% Simulation of TODO 3.2

% Design MPC controller for sys_x
Hx = 0.5;        % Horizon length in seconds
Nx= ceil(Hx/Ts);  % Horizon length in discret steps
mpc_x = MPC_Control_x(sys_x, Ts, Hx);
% Get control input
x0_x=[0,0,0,0]';
x_ref=-5;
ux = mpc_x.get_u(x0_x, x_ref);

[T, X_sub, U_sub] = rocket.simulate(sys_x, x0_x, 15, @mpc_x.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_ref);

%%

% Design MPC controller for sys_y
Hy = 0.5;        % Horizon length in seconds
Ny= ceil(Hy/Ts);  % Horizon length in discret steps
mpc_y = MPC_Control_y(sys_y, Ts, Hy);
% Get control input
x0_y=[0,0,0,0]';
x_ref=-5;
uy = mpc_y.get_u(x0_y, x_ref);

[T, X_sub, U_sub] = rocket.simulate(sys_y, x0_y, 15, @mpc_y.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, x_ref);

%% 

% Design MPC controller for sys_z
Hz = 0.5;        % Horizon length in seconds
Nz= ceil(Hz/Ts);  % Horizon length in discret steps
mpc_z = MPC_Control_z(sys_z, Ts, Hz);
% Get control input
x0_z=[0,0]';
x_ref=-5;
uz = mpc_z.get_u(x0_z, x_ref);

[T, X_sub, U_sub] = rocket.simulate(sys_z, x0_z, 15, @mpc_z.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, x_ref);

%%

% Design MPC controller for sys_roll
Hroll = 0.5;        % Horizon length in seconds
Nroll= ceil(Hroll/Ts);  % Horizon length in discret steps
mpc_roll = MPC_Control_roll(sys_roll, Ts, Hroll);
% Get control input
x0_roll=[0,0]';
x_ref=deg2rad(45);
uroll = mpc_roll.get_u(x0_roll, x_ref);

[T, X_sub, U_sub] = rocket.simulate(sys_roll, x0_roll, 15, @mpc_roll.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, x_ref);
