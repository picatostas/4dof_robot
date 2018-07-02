%% christoffel first order

function C_ijk = Cijk(i,j,k,D)

%syms q1 q2 q3 q4;
%q_v = [q1 q2 q3 q4];

C_ijk = 1/2 * ( diff(D(k,j),q_v(i))+ diff(D(k,i),q_v(j)) - diff(D(i,j),q_v(k)) );

%return C_ijk;
end