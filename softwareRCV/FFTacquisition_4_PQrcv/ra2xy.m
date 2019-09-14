function real = ra2xy(m,a)
% m is magnitude
% a is angle in degree
real = m*cos(a*pi/180)+i*m*sin(a*pi/180);