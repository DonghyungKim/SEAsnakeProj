% This function returns (5x4) Jacobian matrix for the headlook mode
% From x_dot = J*th_dot, we use x = [x y z beta gamma]^T, where 
% => (x,y,z) are the position of E-E frame w.r.t reference frame, 
% => beta, gamma are y-axis (pitch), z-axis (yaw) angle, respectively
% [Inputs]
%  - th :  (4x1) vector of joint angles from 4th module to the 1st module (= head module)
%  - l  :  (3x1) vector of link length from 4th module to the 2nd module. Normally, all the values are equal to the module length
% [Output]
%  - J  :  Jacobian matrix at the pose 'th'

function J = Jacobi_HeadLook(th, l)
    l1 = l(1);
    l2 = l(2);
    l3 = l(3);
    
    J = [-l1*sin(th(1))-l2*sin(th(1))*cos(th(2))-l3*(sin(th(1))*cos(th(2))*sin(th(3))+cos(th(1))*sin(th(3)))    -l2*cos(th(1))*sin(th(2))-l3*cos(th(1))*sin(th(2))*cos(th(3))   -l3*(cos(th(1))*cos(th(2))*sin(th(3))+sin(th(1))*cos(th(3)))        0;
          l1*cos(th(1))+l2*cos(th(1))*cos(th(2))+l3*(cos(th(1))*cos(th(2))*cos(th(3))-sin(th(1))*sin(th(3)))    -l2*sin(th(1))*sin(th(2))-l3*sin(th(1))*sin(th(2))*cos(th(3))    l3*(-sin(th(1))*cos(th(2))*sin(th(3))+cos(th(1))*cos(th(3)))       0;
          0                                                                                                      l2*cos(th(2))+l3*cos(th(2))*cos(th(3))                         -l3*sin(th(2))*sin(th(3))                                           0;
          0                                                                                                      1                                                               0                                                                  1 ;
          1                                                                                                      0                                                               1                                                                  0 ];

end