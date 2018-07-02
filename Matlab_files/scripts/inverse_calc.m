function [theta_1,theta_2,theta_3,theta_4] = inverse_calc(l1,l2,l3,l4,x,y,z,phi)
        if nargin > 1
            %l1 = j1.d;l2 = j2.a ; l3=j3.a; l4=j4.a;
            %q1 
            theta_1 = atan(y/x)
            %cambio de cordenadas para referir respecto del plano
            xe = sqrt( x^2 + y^2);
            ye = z - l1;
            %poscion de la muñeca
            xw =  xe - l4*cos(phi);
            yw =  ye - l4*sin(phi);
            % solucion del subsistema planar 3R
            r = sqrt( xw^2 + yw^2);
            
            alpha = atan(yw/xw);
            
            gamma = acos(( r^2 + l2^2 -l3^2)/(2*l2*r));
            theta_2 = alpha - gamma
            theta_3 = acos(( r^2 -l2^2 -l3^2)/(2*l2*l3))
            theta_4 = phi - theta_2 - theta_3
        end
end