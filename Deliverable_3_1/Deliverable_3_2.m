clear;
clc;
close all;

addpath(fullfile('..', 'src'));

%% TODO 3.1: This file should produce all the plots for the deliverable

Ts       = 1/20; % Sample time
rocket   = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);


%% Simulation of TODO 3.1
% x0 here is the state of the particular subsystem being simulated
% simulate function will add the trim to your MPC controller s input when 
%  you provide it with a linear model

% Design MPC controller for sys_x
Hx = 2.5;        % Horizon length in seconds
Nx= ceil(Hx/Ts);  % Horizon length in discret steps
mpc_x = MPC_Control_x(sys_x, Ts, Hx);
% Get control input
x0_x=[0,0,0,5]';
ux = mpc_x.get_u(x0_x);

[T, X_sub, U_sub] = rocket.simulate(sys_x, x0_x, 10, @mpc_x.get_u, 0); 
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

%%

% Design MPC controller for sys_y
Hy = 3;        % Horizon length in seconds
Ny= ceil(Hy/Ts);  % Horizon length in discret steps
mpc_y = MPC_Control_y(sys_y, Ts, Hy);
% Get control input
x0_y=[0,0,0,5]';
uy = mpc_y.get_u(x0_y);

[T, X_sub, U_sub] = rocket.simulate(sys_y, x0_y, 10, @mpc_y.get_u, 0); 
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);

%%

% Design MPC controller for sys_z
Hz = 5;        % Horizon length in seconds
Nz= ceil(Hz/Ts);  % Horizon length in discret steps
mpc_z = MPC_Control_z(sys_z, Ts, Hz);
% Get control input
x0_z=[0,5]';
uz = mpc_z.get_u(x0_z);

[T, X_sub, U_sub] = rocket.simulate(sys_z, x0_z, 15, @mpc_z.get_u, 0); 
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);

%%

% Design MPC controller for sys_roll
Hroll = 5;        % Horizon length in seconds
Nroll= ceil(Hroll/Ts);  % Horizon length in discret steps
mpc_roll = MPC_Control_roll(sys_roll, Ts, Hroll);
% Get control input
x0_roll=[0,deg2rad(45)]';
uroll = mpc_roll.get_u(x0_roll);

[T, X_sub, U_sub] = rocket.simulate(sys_roll, x0_roll, 10, @mpc_roll.get_u, 0); 
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);