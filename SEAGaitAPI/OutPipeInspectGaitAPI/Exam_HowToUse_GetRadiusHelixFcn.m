clear

r_init = 0.1;
r_max = 1.5;

p = 4*pi;       % period, that is, the time length of single wave (sec)

t = 0; pi;      % curren time (sec)

num_modules = 16;

for i=1:num_modules
    r(i)=GetRadiusHelixForShiftPipe(i, t, r_init, r_max, p, num_modules);
end
    
plot(r,'o-')
grid on
axis([1 16 0 1.1*r_max])

xlabel('Module index')
ylabel('Radius of helix at t=pi')