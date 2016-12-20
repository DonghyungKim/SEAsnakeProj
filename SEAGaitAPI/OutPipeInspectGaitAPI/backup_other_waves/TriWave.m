% This function returns the triangle wave y(x) with y(0) = a
% Ref) https://en.wikipedia.org/wiki/Triangle_wave
% [Inputs]
%  - x: domain of triangle wave function 
%  - a: amplitude
%  - p: period
% [Output]
%  - y: y(x)

function y = TriWave(x, a, p)
    y = (4*a/p)*(abs(mod(x, p) - p/2) - p/4);
end