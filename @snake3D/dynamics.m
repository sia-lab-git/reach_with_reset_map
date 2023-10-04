function dx = dynamics(obj, ~, x, u, ~)
% dot_x_1 = v cos alpha
% dot_x_2 = v sin alpha
% dot alpha = u

if iscell(x)
    dx = cell(length(obj.dims), 1);    
    for i = 1:length(obj.dims)
        dx{i} = dynamics_cell_helper(obj, x, u, obj.dims(i));
    end
else
    dx = zeros(obj.nx, 1);
    dx(1) = obj.v * cos(x(3));
    dx(2) = obj.v * sin(x(3));
    dx(3) = u;
end

end

function dx = dynamics_cell_helper(obj, x, u, dim)
switch dim
    case 1
        dx = obj.v * cos(x{3});
    case 2
        dx = obj.v * sin(x{3});
    case 3
        dx = u;
    otherwise
        error("Dimension index not defined.")
end
end