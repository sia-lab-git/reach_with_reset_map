%% get reset map mode ver
function [reset_map, params] = snake3D_get_reset_map(grid, params)

% define state jump rules for all states, modify this based on your system
x1_post_f = @x_f1;
x2_post_f = @x_f2;
x3_post_f = @x_f3;
state_fcn_arr = {x1_post_f; x2_post_f; x3_post_f};
params.state_fcn_arr = state_fcn_arr;

N = grid.N; % grid num vector
% how many elements in grid, = N(1)*N(2)*N(3)
ind = 1:prod(N); 
% generate 3d indices https://www.mathworks.com/help/matlab/ref/ind2sub.html
% If It's a n-dims system, change to [I1, I2, I3, ... In], 
% and change following
[I1, I2, I3] = ind2sub(N, ind); 

I1_reset = I1; % indectors for reset map dim1
I2_reset = I2; % dim2A
I3_reset = I3; % dim3

for j = ind % loop through the reset map

    i_tmp = [I1(j); I2(j); I3(j)];
    i_post = i_tmp;

    % check if this state satisify the state reset condition
    if resetmap_trigger_event(grid, i_tmp)
        x_tmp = index2state(grid,i_tmp);
        x_post = x_tmp;
        % if so, calculate states after state jumping
        for k = 1:length(state_fcn_arr)
            x_post(k) = state_fcn_arr{k}(x_tmp);
        end
        % convert back to corresponding grid indices
        i_post = state2index(grid, x_post);
    end

    I1_reset(j) = i_post(1);
    I2_reset(j) = i_post(2);
    I3_reset(j) = i_post(3);
end
% reshape back to 1d indices
reset_map = sub2ind(N, I1_reset, I2_reset, I3_reset); 
end


% state jump rule for 1st dim, xpos
function x1_post = x_f1(x)

  if (x(1) <= 0) && (abs(x(3))>pi/2) %left barrier
     x1_post = 10;
  elseif (x(1) >= 10) && (abs(x(3))<pi/2) %right barrier
     x1_post = 0;
  else
    x1_post = x(1);
  end

end


% state jump rule for 2nd dim, ypos
function x2_post = x_f2(x)

  if (x(2) <= 0) && (x(3)<0) %bot barrier
     x2_post = 10;
  elseif (x(2) >= 10) && (x(3)>0) %top barrier
     x2_post = 0;
  else
    x2_post = x(1);
  end

end

% state jump rule for 3rd dim, yaw angle th
function x3_post = x_f3(x)

   x3_post = x(3);

end

function is_reset = resetmap_trigger_event(grid, i_t)
% check if the state reset condition has been triggered for current index
% 1 - reset event triggered, 0 - others
is_reset = 0;
eps = 1e-5;
% get state for this index
x_ref = index2state(grid, i_t);
% check based on current state
  if (x_ref(2)<eps) && (x_ref(3)<eps) %bot barrier
    is_reset = 1;
  elseif (x_ref(2)>10-eps) && (x_ref(3)>eps) %top barrier
    is_reset = 1;
  elseif (x_ref(1)<eps) && (abs(x_ref(3))>pi/2) %left barrier
    is_reset = 1;
  elseif (x_ref(1)>10-eps) && (abs(x_ref(3))<pi/2) %right barrier
    is_reset = 1;
  end
  
end

function i_t = state2index(grid, x_t)
  % convert current state to grid indices
  i_t = ceil((x_t-grid.min)./grid.dx)+1;
  i_t=min([max([ones(grid.dim,1) i_t],[],2) grid.N],[],2); %keep inside index bounds
end

function x_t = index2state(grid, i_t)
% convert current grid index to state
x_t = (i_t-1).*grid.dx + grid.min;
end
