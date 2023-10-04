clear
clc
close all

%% load data
load('snake3D_brt.mat')
data=snake3D.data;
g=snake3D.g;
sys=snake3D.sys;
tau=snake3D.tau2;

params = sys.params;
params.dt = tau(2)-tau(1);

%% init states

x0 = [1;1;-pi/2];

x_arr = x0;
t_arr = [];
ctr_arr = [];
brs_t_ind_arr = [];
dt = tau(2)-tau(1);

for t_n = tau
    t_arr = [t_arr, t_n];
    x_n = x_arr(:,end);
    % check if reaches target
    in_target = check_in_target(x_n,params.center,params.R);
    if in_target == 1
        break;
    end

    % get current slice of brt
    % determine the earliest time that the current state is in the reachable set
    t_earliest_i = floor(tau(end)/dt);
    for t_i = 1:t_earliest_i
        value_at_x = eval_u(g, data(:,:,:, t_i), x_n);
        if value_at_x<0
            t_earliest_i = t_i;
            break;
        end
    end

    brs_t_ind_arr = [brs_t_ind_arr, t_earliest_i];
    brs_at_t = data(:,:,:,t_earliest_i);
    deriv_t = computeGradients(g, brs_at_t);
    deriv_n = eval_u(g, deriv_t, x_n);
    % get optimal control
    opti_ctr_t = sys.optCtrl([],[],deriv_n,'min');
    ctr_arr = [ctr_arr, opti_ctr_t];
    % sim next step
    x_next = sys.updateStateWithResetMap(opti_ctr_t, dt, x_n, [], params);
    x_arr = [x_arr, x_next];

    fprintf('simulating timestep %f\n', t_n);
end

% animation
video=1;
snake_anime(x_arr, g, params,video);


%% check if reaches target area
function [in_target] = check_in_target(car_state, tar_pos, tar_r)
    x = car_state(1);
    y = car_state(2);
    dist = sqrt((x-tar_pos(1))^2+(y-tar_pos(2))^2) - tar_r;

    if dist<0
        in_target = 1;
    else
        in_target = 0;
    end
end
