function uOpt = optCtrl(obj, ~, ~, deriv, uMode)

if ~iscell(deriv)
    deriv = num2cell(deriv);
end

if strcmp(uMode, 'max')
    uOpt = (deriv{3}>=0) * obj.uRange + (deriv{3}<0) * -obj.uRange;
elseif strcmp(uMode, 'min')
    uOpt = (deriv{3}>=0) * -obj.uRange + (deriv{3}<0) * obj.uRange;
else
    error("Unknown uMode");
end

end