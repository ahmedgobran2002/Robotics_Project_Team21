function V = Velocity_kinematics 
coder.extrinsic('syms', 'subs', 'vpa', 'simplify');
syms p1 p2 p3 p4 q1_dot q2_dot q3_dot q4_dot real
syms t q1 q2 q3 q4

J_matrix = Differentiation();

p1 = 0.1 * t;
p2 = 0.3 * t;
p3 = 0.4 * t;
p4 = 0.2 * t;

q1_dot = 0.1;
q2_dot = 0.3;
q3_dot = 0.4;
q4_dot = 0.2;

q_dot = [q1_dot;
         q2_dot;
         q3_dot;
         q4_dot];

tn = 0;

while (tn < 10)   
q1_subs = subs(p1,t,tn);
q2_subs = subs(p2,t,tn);
q3_subs = subs(p3,t,tn);
q4_subs = subs(p4,t,tn);

J_matrix_sub = vpa(subs(J_matrix,[q1,q2,q3,q4],[q1_subs,q2_subs,q3_subs,q4_subs]));

V = simplify(J_matrix_sub * q_dot);
%disp(V);

tn = tn + 0.1;
end

end