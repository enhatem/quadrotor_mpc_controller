t = 0:0.02:3;

ref_z = -1*ones(1,length(t));
ref_T = zeros(1,length(t));

r = [t.', ref_z.', ref_T.'];

save('ref.mat','r')
