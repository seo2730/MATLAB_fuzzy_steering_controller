%% Real State
clear all

scenario = drivingScenario;

roadCenters = ...
    [  0  40  49  50 100  50  49 40 -40 -49 -50 -100  -50  -49  -40    0
     -50 -50 -50 -50   0  50  50 50  50  50  50    0  -50  -50  -50  -50
       0   0   0   0   0   0   0  0   0   0   0    0    0    0    0    0]';
bankAngles = ...
    [  0   0   0   0   0   0   0  0   0   0   0    0    0    0    0    0];

road(scenario, roadCenters, bankAngles, 'lanes', lanespec(2));

rb = roadBoundaries(scenario);

realCar = vehicle(scenario,'ClassID',2,'Position',[0 -50-2.7/2 0],'Yaw',0);

chasePlot(realCar);
fastCar = vehicle(scenario,'ClassID',1);

d = 2.7/2;
h = .45/2;
roadOffset = [ 0  0  0  0  d  0  0  0  0  0  0 -d  0  0  0  0
              -d -d -d -d  0  d  d  d  d  d  d  0 -d -d -d -d
               0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0]';

plot(scenario);
rWayPoints = roadCenters + roadOffset;

% loop around the track four times
rWayPoints = [repmat(rWayPoints(1:end-1,:),5,1); rWayPoints(1,:)];

trajectory(realCar,rWayPoints(:,:), 30);

scenario.SampleTime = 0.02;

scenario.StopTime = 18;
i=1;
while advance(scenario)
  pause(0.001)
  
  vy(i,1) = 0.02*i;
  vy(i,2) = realCar.Velocity(1,2);
  
  psi_dot(i,1) = 0.02*i;
  psi_dot(i,2) = realCar.AngularVelocity(1,3);
  
  vx(i,1) = 0.02*i;
  vx(i,2) = realCar.Velocity(1,1);

  psides_dot(i,1) = 0.02*i;
  
  if i <= 83
    psides_dot(i,2) = 0;
  elseif i > 83 && i <= 352
    psides_dot(i,2) = realCar.Velocity(1,1)/50;      
  elseif i > 352 && i <=519
    psides_dot(i,2) = 0;
  elseif i > 519 && i <=786
    psides_dot(i,2) = realCar.Velocity(1,1)/50;     
  elseif i > 786 && i <=899
    psides_dot(i,2) = 0;
  end
  
  w(i,1) = 0.02*i;
  w(i,2) = psi_dot(i,2) - psides_dot(i,2);
  i = i + 1;
end

%% T-S fuzzy controller
scenario = drivingScenario;

roadCenters = ...
    [  0  40  49  50 100  50  49 40 -40 -49 -50 -100  -50  -49  -40    0
     -50 -50 -50 -50   0  50  50 50  50  50  50    0  -50  -50  -50  -50
       0   0   0   0   0   0   0  0   0   0   0    0    0    0    0    0]';
bankAngles = ...
    [  0   0   0   0   0   0   0  0   0   0   0    0    0    0    0    0];

road(scenario, roadCenters, bankAngles, 'lanes', lanespec(2));

rb = roadBoundaries(scenario);

realCar = vehicle(scenario,'ClassID',1,'Position',[0 -50-2.7/2 0],'Yaw',0);

d = 2.7/2;
h = .45/2;
roadOffset = [ 0  0  0  0  d  0  0  0  0  0  0 -d  0  0  0  0
              -d -d -d -d  0  d  d  d  d  d  d  0 -d -d -d -d
               0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0]';

%%%% fuzzy modeling %%%%
lf = 1.4; lr = 1.4;
Caf = 10000; Car = 10000; Fyf = 100; Fyr = 100; Bmin = 0.0001; Bmax = pi;
m = 1800;
v = 30;
Iz = 50000;%m*(4.7^2+1.8^2)/12;

C  = [1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      0 0 0 1];

% Rule 1
A1 = [0                           1                     0                              0;
      0        (-2*Caf-2*Car)/(m*v)       (2*Caf+2*Car)/m      (-2*lf*Caf-2*lr*Car)/(m*v);
      0                           0                     0                               1;
      0 (-2*lf*Caf-2*lr*Car)/(Iz*v) (2*lf*Caf-2*lr*Car)/Iz (-2*lf*lf*Caf-2*lr*lr*Car)/(Iz*v)];

B1 = [0 2*Caf/m 0 2*lf*Caf/Iz]';

G1 = [0 -v-(2*lf*Caf-2*lr*Car)/(m*v) 0 -(2*lf*lf*Caf+2*lr*lr*Car)/(Iz*v)]';

% Rule 2
A2 = [0                           1                     0                              0;
      0        (-2*Caf-2*Car)/(m*v*cosd(89.9))       (2*Caf+2*Car)/m      (-2*lf*Caf-2*lr*Car)/(m*v*cosd(89.9));
      0                           0                     0                               1;
      0 (-2*lf*Caf-2*lr*Car)/(Iz*v*cosd(89.9)) (2*lf*Caf-2*lr*Car)/Iz (-2*lf*lf*Caf-2*lr*lr*Car)/(Iz*v*cosd(89.9))];

B2 = [0 2*Caf/m 0 2*lf*Caf/Iz]';

G2 = [0 -v*cosd(89.9)-(2*lf*Caf-2*lr*Car)/(m*v*cosd(89.9)) 0 -(2*lf*lf*Caf+2*lr*lr*Car)/(Iz*v*cosd(89.9))]';

% LMI
setlmis([])

n = 4; 
r = 1.5;
X = lmivar(1, [n, 1]);      
Y1 = lmivar(2, [1, n]);
Y2 = lmivar(2, [1, n]);

% LMI condition 1
lmiterm([1 1 1 X], A1, 1, 's')      % A1*X + X*A1'
lmiterm([1 1 1 Y1], -B1, 1, 's')    % -B1*Y1 - Y1'*B1'
lmiterm([1 2 2 0],-r^2);
lmiterm([1 2 1 0],G1');
lmiterm([1 3 1 X],C,1);
lmiterm([1 3 3 0],-1);
% LMI condition 2
lmiterm([2 1 1 X], A2, 1, 's')      % A2*X + X*A2'
lmiterm([2 1 1 Y1], -B2, 1, 's')    % -B2*Y1 - Y1'*B2'
lmiterm([2 2 2 0],-r^2);
lmiterm([2 2 1 0],G2');
lmiterm([2 3 1 X],C,1);
lmiterm([2 3 3 0],-1);
% LMI condition 3
lmiterm([3 1 1 X], A1, 1, 's')      % A1*X + X*A1'
lmiterm([3 1 1 Y2], -B1, 1, 's')    % -B1*Y2 - Y2'*B1'
lmiterm([3 2 2 0],-r^2);
lmiterm([3 2 1 0],G1');
lmiterm([3 3 1 X],C,1);
lmiterm([3 3 3 0],-1);
% LMI condition 4
lmiterm([4 1 1 X], A2, 1, 's')      % A2*X + X*A2'
lmiterm([4 1 1 Y2], -B2, 1, 's')    % -B2*Y2 - Y2'*B2'
lmiterm([4 2 2 0],-r^2);
lmiterm([4 2 1 0],G2');
lmiterm([4 3 1 X],C,1);
lmiterm([4 3 3 0],-1);
% LMI condition 5
lmiterm([-5 1 1 X], 1, 1)   % specify right side

lmi_sys = getlmis;

[tmin, xfeas] = feasp(lmi_sys);

% if tmin < 0
    Xs = dec2mat(lmi_sys, xfeas, X);
    Ys1 = dec2mat(lmi_sys, xfeas, Y1);
    Ys2 = dec2mat(lmi_sys, xfeas, Y2);
% end

F1 = Ys1 * inv(Xs);
F2 = Ys2 * inv(Xs);
P = inv(Xs); 

%%
fuzzyCar = vehicle(scenario,'ClassID',1,'Position',[0 -50-2.7/2 0],'Yaw',0);
chasePlot(fuzzyCar);

plot(scenario);
rWayPoints = roadCenters + roadOffset;

% loop around the track four times
rWayPoints = [repmat(rWayPoints(1:end-1,:),5,1); rWayPoints(1,:)];

trajectory(realCar,rWayPoints(:,:), 30);

scenario.SampleTime = 0.02;
fuzzyCar.Velocity = [30 0 0];

scenario.StopTime = 18;

k=1;
while advance(scenario)
  pause(0.001)
  fuzzyCar.Yaw = realCar.Yaw;
  fuzzyCar.Velocity = [vx(k,2), out.pos.signals.values(k,1),0];
  fuzzyCar.Position = fuzzyCar.Position + [vx(k,2)*0.02, -out.pos.signals.values(k,1)*0.02,0];
  R(k) = getframe(gcf);
  k = k+1;
end

V = VideoWriter('fuzzy_controller.mp4','MPEG-4');