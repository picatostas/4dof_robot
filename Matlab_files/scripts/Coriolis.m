%% Coriolis matrix index

function Ckj = Coriolis(k,j,D)
% Faltaria  multiplicar por la velocidad con indice i
syms q1 q2 q3 q4;
q_v = [q1 q2 q3 q4];
aux = 0;
for i = 1:4
aux =aux + 1/2 * ( diff(D(k,j),q_v(i))+ diff(D(k,i),q_v(j)) - diff(D(i,j),q_v(k)) );
end
Ckj = aux;
%return C_ijk;
end