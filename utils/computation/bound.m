function x = bound(x, minval, maxval)
%Bounds x between minval and maxval
%If x is a vector then minval and maxval must either be scalars
%or vectors the same size as x
    one = ones(size(x));
    x = min(maxval.*one, max(minval.*one, x));
end