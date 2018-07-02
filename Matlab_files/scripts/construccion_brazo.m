%%
% Longitudes de los eslabones del brazo
% topologia tipo scorbot con trastacion intermedia entre 
% pitch y roll 
d0 = 115.5 ; 
d1 = 110;
d2 = 110; 
d3 = 70;
d4 = 110;
%%
% mascara necesaria para usar el comando ikine 
M = [1 1 1 1 1 0 ];

%%
l1 = link([pi/2 0 0 d0 ]);
l2 = link([0 d1 0 0  0 2]);
l3 = link([0 d2 0 0 0 -pi/2 ]);
l4 = link([-pi/2 0 0 0 0 -3*pi/4 ]);
l5 = link([0 0 0 d3+d4]);
%% 
brazo = robot({ l1 l2 l3 l4 l5 });
drivebot(brazo);

