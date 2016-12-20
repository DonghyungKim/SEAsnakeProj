% This function returns the modified triangle wave y(x) with y(0) = a
% Ref) https://en.wikipedia.org/wiki/Triangle_wave
% [Inputs]
%  - x: domain of triangle wave function 
%  - a: amplitude
%  - p: period
% [Output]
%  - y: y(x)

function y = ModTrapWave(x, a, p, alpha)
    y = a*trapmf(x + p/2, [0  p/2  p/2+alpha  p+alpha]);
end