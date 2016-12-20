clear

x = 0:0.01:50;
a = 1;
p = 4*pi;

y = TriWave(x, a, p);

plot(x, y)
grid on
xlabel('x')
ylabel('y(x)')