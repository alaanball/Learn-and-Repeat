% dist bw the wheels and radius of the wheels in cm
d0 = 10;
r = 2;

% initial positions of the wheels as column vectors

xr0 = [d0/2; 0]; 
xl0 = [-d0/2; 0];

% counter clockwise rotation by 90 matrix

M = [0, -1; 1, 0];

% time step, final time, initial time
dt = 0.01;
tfin = 10;
t0 = 0;

T = t0: dt: tfin;

% angular velocities of the motors with units so thar r * w gives linear velocities
% in cm per second

wr = (2*pi /60) * 25 * ones(1, length(T)); % 25 rpm
wl = (2*pi /60) * 24 * ones(1, length(T)); % 24 rpm

%wr = (2*pi /60) * [25 * ones(1, length(T)/2), 23 * ones(1, length(T)/2)];
%wl = (2*pi /60) * [24 * ones(1, length(T)/2), 25 * ones(1, length(T)/2)];

% position column vectors of the wheels at different times initialized as zero for time 
% greater than 0

xr = [xr0(1), zeros(1, length(T) -  1); xr0(2), zeros(1, length(T) -  1)];
xl = [xl0(1), zeros(1, length(T) -  1); xl0(2), zeros(1, length(T) -  1)];
 
% calculating the position of the wheels
 
for k = 2 : length(T)
  
  % dir is unit vector perpendicular to the vector joining the wheels
  % it is also the direction of the linear velocities of both the wheels
  
  dir = M * [xr(1, k -1) - xl(1, k -1); xr(2, k -1) - xl(2, k -1)] / d0;
  dirdel = dir * dt;
  
  % integration of each coordinate
  
  for i = 1 : 2
    
    % x[t_k] = x[t_(k-1)] + integral from t_k to t_(k-1) of the velocity vector
    
    xr(i, k) = xr(i, k-1) + r * wr(k-1) *dirdel(i);
    xl(i, k) = xl(i, k-1) + r * wl(k-1) *dirdel(i);
    
  endfor
  
endfor
 
% xc is the coordinate of the centre of the wheels
 
xc = (xr + xl)/ 2;
 
% d is the distance bw the wheels at different times. 
% This was only calculated to verify the correctness of the simulation
% If the simulation is correct, the distance bw the wheels should be constant
 
d = (xr(1,:) - xl(1,:)) .* (xr(1,:) - xl(1,:)) + (xr(2,:) - xl(2,:)) .* (xr(2,:) - xl(2,:));
d = sqrt(d);
 
% plots
 
figure(1);
axis( "equal");
plot(xr(1, 1: length(T)), xr(2, 1: length(T)), 'r');
hold on;
plot(xl(1, 1: length(T)), xl(2, 1: length(T)), 'b');
hold on;
plot(xc(1, 1: length(T)), xc(2, 1: length(T)), 'm');

figure(2);
plot(T, d);