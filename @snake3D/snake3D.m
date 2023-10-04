classdef snake3D < DynSys
    properties
        uRange     
        v
        params
        dims        
    end
    methods
      function obj = snake3D(x,v,uRange,params)            

            obj.uRange=uRange;
            obj.v = v;
            obj.params = params;
            
            obj.nu = 1;
            obj.nd = 0;
            obj.x = x;
            obj.dims  = 1:3;
            obj.nx = length(obj.dims);
        end
    end
end

                