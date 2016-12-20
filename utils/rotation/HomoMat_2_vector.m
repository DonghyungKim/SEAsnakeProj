function x = HomoMat_2_vector(Mat)
    if (size(Mat,1) == 4)
        x(1) = Mat(1,4);
        x(2) = Mat(2,4);
        x(3) = Mat(3,4);
    elseif (size(Mat,1) == 3)
        x(1) = 0;
        x(2) = 0;
        x(3) = 0;
    end
    
	x(5) = atan2( -Mat(3,1), sqrt(Mat(1,1)^2 + Mat(2,1)^2 )  );
	if ( (x(5)>0.0) && (cos(x(5))<=0.001) ) 
		x(6) = 0;
		x(4) = atan2(Mat(1,2), Mat(2,2));
    else
		if ( (x(5)<0.0) && (cos(x(5))<=0.001) )
			x(6) = 0;
			x(4) = -atan2(Mat(1,2), Mat(2,2));
        else
			x(6) = atan2( Mat(2,1)/cos(x(5)), Mat(1,1)/cos(x(5)) );
			x(4) = atan2( Mat(3,2)/cos(x(5)), Mat(3,3)/cos(x(5)) );
        end
    end
            
	if (x(4) < -pi)
		x(4) = x(4) + 2.0*pi; % Force the orientation into (-pi,pi)
    elseif (x(4) >= pi)
		x(4) = x(4) - 2.0*pi;
    end
	if (x(5) < -pi)
		x(5) = x(5) + 2.0*pi; % Force the orientation into (-pi,pi)
    elseif (x(5) >= pi)
		x(5) = x(5) - 2.0*pi;
    end
	if (x(6) < -pi)
		x(6) = x(6) + 2.0*pi; % Force the orientation into (-pi,pi)
    elseif (x(6) >= pi)
		x(6) = x(6) - 2.0*pi;
    end
    
end