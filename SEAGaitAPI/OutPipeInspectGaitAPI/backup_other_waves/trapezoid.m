function s = trapezoid(t)
%TREPEZOID Trapezoid wave generation.
    %   TRAPEZOID(T) generates a square wave with period 2*Pi for the
    %   elements of time vector T.  TRAPEZOID(T) is like SQUARE(T), only
    %   it creates a square wave with peaks of +1 to -1 instead of
    %   a sine wave.
    %   
    %   For example, generate a 30 Hz square wave:
    %        t = 0:.0001:.0625;
    %        y = square(2*pi*30*t);, plot(t,y)
    %
    %   I pretty much stole this from square, so see that.
    %   See also SQUARE

    %   Author(s): Jordan Firth, 2 Sep 2010
    %   $Revision: 0.0 $  $Date: 2007/12/14 15:06:21 $

    % compute values of t normalized to (pi/4,9*pi/4)
%      tmp = mod(t-pi/4,2*pi);


	tmp = mod(t+pi/4,2*pi);


    a = (tmp < pi/2);
    b = (tmp >= pi/2 & tmp < pi);
    c = (tmp >= pi & tmp < 3*pi/2);

    rise = 2*tmp/pi;
    fall = -2*(tmp-pi)/pi+1;

    nodd = a.*rise + b + c.*fall;

    s = 2*nodd-1;

end