function Q_DOT = Velocity_kinematics_inverse
coder.extrinsic('syms', 'subs', 'vpa', 'simplify');
syms X_dot Y_dot Z_dot real
syms p1 p2 p3 p4 q1_dot q2_dot q3_dot q4_dot real
syms t q1 q2 q3 q4

J_pseudo_inv = J_inv();
V = Velocity_kinematics();
J_matrix = Differentiation();

p1 = 0.1 * t;
p2 = 0.3 * t;
p3 = 0.4 * t;
p4 = 0.2 * t;

X_dot = V(1);
Y_dot = V(2);
Z_dot = V(3);

Q_DOT = [q1_dot; 
         q2_dot;
         q3_dot;
         q4_dot];

tn = 0;

while (tn < 10)   
q1_subs = subs(p1,t,tn);
q2_subs = subs(p2,t,tn);
q3_subs = subs(p3,t,tn);
q4_subs = subs(p4,t,tn);

J_mat_sub = vpa(subs(J_matrix,[q1,q2,q3,q4],[q1_subs,q2_subs,q3_subs,q4_subs]));
J_inv_sub = pinv(J_mat_sub);

%J_inv_sub = vpa(subs(J_pseudo_inv,[q1,q2,q3,q4],[q1_subs,q2_subs,q3_subs,q4_subs]));

Q_DOT = simplify(J_inv_sub * V);
%disp(Q_DOT);

tn = tn + 0.1;
end
end