%%
% Longitudes de los eslabones del brazo
% 
% 

%%
clc 
clear all
%load('paramidenmot.mat')
joint ;%     alfa     a    theta      d 
l1 = 115e-3;
l2 = 75e-3;
l3 = 80e-3;
l4 = 89e-3;
j1 = joint(  pi/2,   0,       0, l1);
j2 = joint(     0, l2,    0,     0);
j3 = joint(     0, l3,   0,     0);
j4 = joint(0,   l4,   0,     0);

%%
% mascara necesaria para usar el comando ikine 
M = [1 1 1 1 1 0 ];

%%
lin1 = link([j1.alfa j1.a 0 j1.d 0 j1.theta ]);
lin2 = link([j2.alfa j2.a 0 j2.d 0 j2.theta ]);
lin3 = link([j3.alfa j3.a 0 j3.d 0 j3.theta ]);
lin4 = link([j4.alfa j4.a 0 j4.d 0 j4.theta ]);

brazo_1 = robot({ lin1 lin2 lin3 lin4});


%% 

drivebot(brazo_1);
%% cinematica directa metodo dos
% Las he calculado a mano porque al usar el symbolic toolbox no me calcula
% cosas my obvias
syms q1 q2 q3 q4 ;
q_v = [q1 q2 q3 q4];
%q1= 1.75 ,q2= pi/2, q3= pi/3 , q4=pi/3 ,q5=pi/3;
%q1 = pi/4 , q2 = pi/4, q3 = -pi/2 , q4 = -pi/4
T01 = [             cos(q1) 0 sin(q1) 0 ;  sin(q1)       0 -cos(q1) 0;         0 1 0 j1.d;  0 0 0 1];
T12 = [cos(q2) -sin(q2) 0 j2.a*cos(q2);  sin(q2) cos(q2) 0 j2.a*sin(q2);       0 0 1 0;  0 0 0 1];
T23 = [cos(q3) -sin(q3) 0 j3.a*cos(q3);  sin(q3) cos(q3) 0 j3.a*sin(q3);       0 0 1 0;  0 0 0 1];
T34 = [cos(q4) -sin(q4) 0 j4.a*cos(q4);  sin(q4) cos(q4) 0 j4.a*sin(q4);       0 0 1 0;  0 0 0 1];
T04_calc = [cos(q1)*cos(q2+q3+q4) -cos(q1)*sin(q2+q3+q4) sin(q1) (j2.a*cos(q2)*cos(q1)+j4.a*cos(q1)*cos(q2+q3+q4) + j3.a*cos(q1)*cos(q2+q3));sin(q1)*cos(q2+q3+q4) -sin(q1)*sin(q2+q3+q4) -cos(q1) (j2.a*cos(q2)*sin(q1)+j4.a*sin(q1)*cos(q2+q3+q4) + j3.a*sin(q1)*cos(q2+q3));sin(q2+q3+q4) cos(q2+q3+q4) 0 (j2.a*sin(q2) + j3.a*sin(q2+q3) + j4.a*sin(q2+q3+q4) + j1.d);0 0 0 1];

T04_sim= T01*T12*T23*T34;
T03 = T01*T12*T23;
T02 = T01*T12;
T13 = T12*T23;





%% 

% matrices de inercia para el caso de calcularla respecto al eje de
% coordenadas donde esta la articulacion
%               /Ixx Ixy Ixz\
% Donde Ii =    |Iyx Iyy Iyz|
%               \Izx Izy Izz/
% Siendo simetrica definida positiva expresada en Kgm^2
% obtenidas en catia con los stl de las articulaciones
% donde para lo obtenido en catia la matriz de inercia es
%               /Ixx -Ixy -Ixz\
% Donde Ii =    |-Iyx Iyy -Iyz|
%               \-Izx -Izy Izz/

%I1 = [3.958e-4 -3.168e-8 -2.064e-4;-3.168e-8 7.936e-4 -1.18e-6;-2.064e-4 -1.18e-6 5.33e-4];
%I2 = [3.017e-5 -1.822e-6 4.404e-6;-1.822e-6 4.129e-4 -6.142e-8;4.404e-6 -6.142e-8 3.942e-4];
%I3 = [2.817e-5 2.549e-6 6.631e-6;2.549e-6 4.085e-4 -1.533e-8;6.631e-6 -1.533e-8 3.897e-4];
%I4 = [2.173e-5 2.124e-5 5.014e-6;2.124e-5 1.421e-4 1.125e-6;5.014e-6 1.125e-6 1.37e-4];
%En caso de poner el centro en el cdm 
Ix1 = 1.504e-4; Iy1 =  5.34e-4;Iz1 = 5.476e-4;
Ix2 = 3.009e-5; Iy2 = 7.548e-4;Iz2 = 9.591e-5;
Ix3 = 2.733e-5; Iy3 = 6.248e-5;Iz3 = 8.046e-5;
Ix4 = 1.773e-5; Iy4 = 3.101e-5;Iz4 = 3.962e-5;
%%
l1 = 115e-3;
l2 = 80e-3;
l3 = 80e-3;
l4 = 89e-3;
%masas de los eslabones
m1 = 217e-3; m2 = 91e-3; m3 = 88e-3; m4 = 56e-3;
% distancias de los centros de masa a las articulaciones
r1 = 80.11e-3; r2 = 59.16e-3; r3 = 61.08e-3; r4 = 43.73e-3;
g = 9.81;
% alturas de los centros de gravedad
h1 = r1;
h2 = l1 + r2*sin(q2);
h3 = l1 + l2*sin(q2) + r3*sin(q2 + q3);
h4 = l1 + l2*sin(q2) + l3*sin(q2 + q3) + r4*sin(q2 + q3 + q4);
% energia potencial 
p1  = m1*h1*9.81;
p2  = m2*h2*9.81;
p3  = m3*h3*9.81;
p4  = m4*h4*9.81;
% potencial gravitatorio
V_theta = g*(m1*h1 + m2*h2 + m3*h3 + m4*h4);
g_1 = vpa(simplify(diff(V_theta,q1)),4);
g_2 = vpa(simplify(diff(V_theta,q2)),4);
g_3 = vpa(simplify(diff(V_theta,q3)),4);
g_4 = vpa(simplify(diff(V_theta,q4)),4);

g_theta = [g_1;g_2;g_3;g_4];

%%
% Tensores de inercia de los eslabones
I1 = [Ix1 0 0;0 Iy1 0;0 0 Iz1]; %#ok<*NOPTS>
I2 = [Ix2 0 0;0 Iy2 0;0 0 Iz2];
I3 = [Ix3 0 0;0 Iy3 0;0 0 Iz3];
I4 = [Ix4 0 0;0 Iy4 0;0 0 Iz4];

%q1 = 0 , q2 = 0, q3 = 0 , q4 = 0
%q1 = pi/4 , q2 = pi/4, q3 = -pi/2 , q4 = -pi/4;
D11 = Ix2*sin(q2)^2 + Ix3*sin(q2+q3)^2 + Ix4*sin(q2+q3+q4)^2 + Iz1 + Iz2*cos(q2)^2 + Iz3*cos(q2+q3)^2 + Iz4*cos(q2+q3+q4)^2 + m2*r2^2*cos(q2)^2 +m3*(r3*cos(q2+q3)+l2*cos(q2))^2 + m4*(l3*cos(q2+q3) + l2*cos(q2) + r4*cos(q2 + q3+q4))^2;
D12 = 0;
D13 = 0;
D14 = 0;
D21 = 0;
D22 =Iy2 + Iy3 + Iy4 + m2*r2^2+ m3*l2^2*sin(q3)^2 + m3*(r3 + l2*cos(q3))^2 +m4*(l2*sin(q2+q3) + l3*sin(q4))^2 + m4*(r4 + l2*cos(q3+q4) + l3*cos(q4))^2;
D23 = Iy3 + Iy4 + m3*r3*(r3 + l2*cos(q3)) + m4*l3*sin(q4)*(l2*sin(q2 +q3) + l3*sin(q4)) +m4*(r4 + l3*cos(q4))*(r4 + l2*cos(q3+q4) + l3*cos(q4)); 
D24 = Iy4 + m4*r4*(r4 + l2*cos(q3+q4) + l3*cos(q4));
D31 = 0;
D32 = 0;
D33 = Iy3 + Iy4 + m3*r3^2 + m4*l3^2*sin(q4)^2+ m4*(r4 + l3*cos(q4))^2;
D34 = Iy4 + m4*r4*(r4 + l3*cos(q4));
D41 = 0;
D42 = 0;
D43 = 0;
D44 = Iy4 + m4*r4^2;

D = [D11 D12 D13 D14;D21 D22 D23 D24;D31 D32 D33 D34;D41 D42 D43 D44];

%%
C11 = vpa(simplify(Coriolis(1,1,D),'IgnoreAnalyticConstraints',true),4);
C12 = vpa(simplify(Coriolis(1,2,D),'IgnoreAnalyticConstraints',true),4);
C13 = vpa(simplify(Coriolis(1,3,D),'IgnoreAnalyticConstraints',true),4);
C14 = vpa(simplify(Coriolis(1,4,D),'IgnoreAnalyticConstraints',true),4);
C22 = vpa(simplify(Coriolis(2,2,D),'IgnoreAnalyticConstraints',true),4);
C23 = vpa(simplify(Coriolis(2,3,D),'IgnoreAnalyticConstraints',true),4);
C24 = vpa(simplify(Coriolis(2,4,D),'IgnoreAnalyticConstraints',true),4);
C33 = vpa(simplify(Coriolis(3,3,D),'IgnoreAnalyticConstraints',true),4);
C34 = vpa(simplify(Coriolis(3,4,D),'IgnoreAnalyticConstraints',true),4);
C44 = vpa(simplify(Coriolis(4,4,D),'IgnoreAnalyticConstraints',true),4);


%sym C_test
%for i = 1:4
%    for j = 1:4
%        C_test(i,j) = simplify(Coriolis(i,j,D),'IgnoreAnalyticConstraints',true);
%    end
%end

C = vpa([C11 C12 C13 C14;C12 C22 C23 C24;C13 C23 C33 C34;C14 C24 C34 C44],4);






