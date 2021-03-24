seed=300;

rng(seed);

t = 0: 0.01 : 6;

y = wgn(length(t),1,0);


plot(t,y)

disturbance = [t.', y];

save('disturbance.mat','disturbance')