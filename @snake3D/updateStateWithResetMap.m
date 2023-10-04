function x_out = updateStateWithResetMap(obj, u, T, x0, d, params)

% If no state is specified, use current state
if nargin < 4 || isempty(x0)
  x0 = obj.x;
end
% If time horizon is 0, return initial state
if T == 0
  x_out = x0;
  return
end
% Default disturbance
if nargin < 5
  d = [];
end


odeOpts = odeset('Events', @(t,x)sys_reset_event(t,x));
% sim next state
if isempty(d) % Check whether there's disturbance
  [ts, x] = ode113(@(t,x) obj.dynamics(t, x, u), [0 T], x0, odeOpts);
else
  [~, x] = ode113(@(t,x) obj.dynamics(t, x, u, d), [0 T], x0, odeOpts);
end
% get next state, (last one)
x_now = x(end, :)';
% check if triggers state reset
value_end = sys_reset_event(ts(end), x_now);%%%%give full state

if value_end < 0
    % Apply state reset
    x_post = zeros(3, 1);
    for i=1:length(params.state_fcn_arr)
        x_post(i) = params.state_fcn_arr{i}(x_now);
    end
    obj.x = x_post;
else
    obj.x = x_now;
end

% check if heading angle goes beyond range -pi~pi
if obj.x(3) > pi
    obj.x(3) = obj.x(3) - 2*pi;
elseif obj.x(3) < -pi
    obj.x(3) = obj.x(3) + 2*pi;
end

x_out = obj.x;
% Update the state, state history, control, and control history
obj.u = u;
obj.xhist = cat(2, obj.xhist, obj.x);
obj.uhist = cat(2, obj.uhist, u);
end

function [value, isterminal, direction] = sys_reset_event(t, x_ref)
% check if the current state triggers the reset event
% -1 - triggers, 0 - others
value = 0;
eps = 1e-5;
% check based on current state
if (x_ref(2)<eps) && (x_ref(3)<eps) %bot barrier
  value = -1;
elseif (x_ref(2)>10-eps) && (x_ref(3)>eps) %top barrier
  value = -1;
elseif (x_ref(1)<eps) && (abs(x_ref(3))>pi/2) %left barrier
  value = -1;
elseif (x_ref(1)>10-eps) && (abs(x_ref(3))<pi/2) %right barrier
  value = -1;
end

%placeholders
isterminal = 1;
direction = 1;

end