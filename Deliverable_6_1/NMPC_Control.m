function opti_eval = NMPC_Control(rocket, H)

import casadi.*
opti = casadi.Opti(); % Optimization problem

N = ceil(H/rocket.Ts); % MPC horizon
nx = 12; % Number of states
nu = 4;  % Number of inputs

% Decision variables (symbolic)
X_sym = opti.variable(nx, N); % state trajectory
U_sym = opti.variable(nu, N-1);   % control trajectory)

% Parameters (symbolic)
x0_sym  = opti.parameter(nx, 1);  % initial state
ref_sym = opti.parameter(4, 1);   % target position


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

f = @(x,u) rocket.f(x,u); %function handler for rocket dynamics

%Steady state parameters
Xs  = opti.variable(nx, 1);  % initial state
Us = opti.variable(nu, 1);   % target position

% ---- objective ---------
cost=0;

Q=eye(12); 
Q(10,10) = 1125; Q(11,11) = 1125; Q(12,12) = 1125;
Q(7,7)=250; Q(8,8)=250; Q(9,9)=250;
Q(6,6)=500;
R = eye(4);

for k = 1:N-1
    cost = cost + (U_sym(:,k)-Us)'*R*(U_sym(:,k)-Us);
    cost = cost + (X_sym(:,k)-Xs)'*Q*(X_sym(:,k)-Xs);
end
cost = cost + (X_sym(:,end)-Xs)'*Q*(X_sym(:,end)-Xs);
opti.minimize(cost)




% ---- Dynamics constraints -------
h=0.2;
for k=1:N-1 % loop over control intervals
  opti.subject_to(X_sym(:,k+1) == RK4(X_sym(:,k), U_sym(:,k),h, f));
end


% initial state
opti.subject_to(X_sym(:,1) == x0_sym);

%------------state constraints-------------------------- 

% contraindre Beta pour eviter la singularité
opti.subject_to(deg2rad(-85) <= X_sym(5,:) <= deg2rad(85))

%------------------input constraints--------------------
opti.subject_to(deg2rad(-15) <= U_sym(1,:) <= deg2rad(15))  % contrainte sur delta 1
opti.subject_to(deg2rad(-15) <= U_sym(2,:) <= deg2rad(15))  % contrainte sur delta 2
opti.subject_to(    20     <= U_sym(3,:) <= 80)          % contrainte sur Pavg
opti.subject_to(    -20     <= U_sym(4,:) <= 20)          % contrainte sur Pdiff

%------------------steady-state constraints--------------------
opti.subject_to(deg2rad(-85) <= Xs(5,:) <= deg2rad(85))
opti.subject_to(deg2rad(-15) <= Us(1,:) <= deg2rad(15))  % contrainte sur delta 1
opti.subject_to(deg2rad(-15) <= Us(2,:) <= deg2rad(15))  % contrainte sur delta 2
opti.subject_to(    20     <= Us(3,:) <= 80)          % contrainte sur Pavg
opti.subject_to(    -20     <= Us(4,:) <= 20)          % contrainte sur Pdiff
opti.subject_to(Xs(6)==ref_sym(4));
opti.subject_to(Xs(10)==ref_sym(1));
opti.subject_to(Xs(11)==ref_sym(2));
opti.subject_to(Xs(12)==ref_sym(3));
opti.subject_to(Xs == RK4(Xs, Us,h,f))

% YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ---- Setup solver ------
ops = struct('ipopt', struct('print_level', 0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);

% Create function to solve and evaluate opti
opti_eval = @(x0_, ref_) solve(x0_, ref_, opti, x0_sym, ref_sym, U_sym);
end

function u = solve(x0, ref, opti, x0_sym, ref_sym, U_sym)

% ---- Set the initial state and reference ----
opti.set_value(x0_sym, x0);
opti.set_value(ref_sym, ref);

% ---- Solve the optimization problem ----
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');

u = opti.value(U_sym(:,1));

% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end
