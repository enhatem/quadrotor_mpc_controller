
%% Stationary trajectory
t = 0:0.01:6;
ref_x = zeros(1,length(t));
ref_y = zeros(1,length(t));
ref_z = -1*ones(1,length(t));
ref_T = zeros(1,length(t)); % F1 F2 F3 F4

r = [t.', ref_x.', ref_y.', ref_z.', ref_T.', ref_T.', ref_T.', ref_T.'];

save('ref.mat','r')


%% Vertical trajectory

t1 = 0 : 0.01 : 0.5;
t2 = 0.51 : 0.01 : 2.5;
t3 = 2.51 : 0.01 : 5;
t4 = 5.01 : 0.01 : 6;
%%
z1 = -0.3*ones(1,length(t1));
z2 = -2.0*ones(1,length(t2));
z3 = -3.0*ones(1,length(t3));
z4 = -1*ones(1,length(t4));
%%
t = [t1, t2, t3, t4];
ref_z = [ z1, z2, z3, z4];
ref_T = zeros(1,length(t));

figure
plot (t,-ref_z);
grid on
%%
r = [t.', ref_z.', ref_T.'];

save('ref.mat','r')

