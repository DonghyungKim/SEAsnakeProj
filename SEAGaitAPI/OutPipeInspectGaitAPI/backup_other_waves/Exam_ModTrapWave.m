clear

p=4*pi;
alpha = 1;
a=2;


t_init = 0;
t=t_init:0.01:p/2+alpha+t_init ;


y=ModTrapWave(t, a, p, alpha);

plot(t, y)
axis([t_init   p/2+alpha+t_init    0   a])