classdef joint
    properties
        alfa
        a
        theta
        d
      %  q
    end
methods
    function joint =joint(alfa_in,a_in,theta_in,d_in)
        if nargin > 1
            
        joint.alfa = alfa_in;
        joint.a = a_in;
        joint.theta = theta_in;
        joint.d = d_in;
        end
        
    end
end

end
