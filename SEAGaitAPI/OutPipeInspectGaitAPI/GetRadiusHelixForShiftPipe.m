function r=GetRadiusHelixForShiftPipe(i, t, r_init, r_max, p, num_modules)
	a = (r_max - r_init)/2;
    T = t + ((p/2)/(num_modules-1))*(i-1);   
    r = a*cos((2*pi/p)*T) + a + r_init;
end