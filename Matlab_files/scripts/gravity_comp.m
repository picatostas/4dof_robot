function [g1,g2,g3,g4] = gravity_comp(q1,q2,q3,q4)

g1 = 0.0;

g2 = 0.02402*cos(q2 + q3 + q4) + 0.09668*cos(q2 + q3) + 0.1658*cos(q2);

g3 = 0.02402*cos(q2 + q3 + q4) + 0.09668*cos(q2 + q3);

g4 = 0.02402*cos(q2 + q3 + q4);









end