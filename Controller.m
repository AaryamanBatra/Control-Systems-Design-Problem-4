function func = Controller
% INTERFACE
%
%   sensors
%       .e_lateral      (error in lateral position relative to road)
%       .e_heading      (error in heading relative to road)
%       .wR             (angular velocity of right wheel)
%       .wL             (angular velocity of left wheel)
%       .r_road         (signed radius of curvature of road - to find the
%                        corresponding turning rate for a given forward
%                        speed: w_road = v_road/sensors.r_road)
%
%   references
%       (none)
%
%   parameters
%       .tStep      (time step)
%       .tauMax     (maximum wheel torque)
%       .roadwidth  (width of road - half on each side of centerline)
%       .symEOM     (nonlinear EOMs in symbolic form)
%       .numEOM     (nonlinear EOMs in numeric form)
%       .b          (distance between the two wheels)
%       .r          (radius of each wheel)
%
%   data
%       .whatever   (yours to define - put whatever you want into "data")
%
%   actuators
%       .tauR       (right wheel torque)
%       .tauL       (left wheel torque)

% Do not modify this part of the function.
func.init = @initControlSystem;
func.run = @runControlSystem;

% If you want sensor data to be delayed, set func.iDelay to some
% non-negative integer (this is the number of time steps of delay).
func.iDelay = 0;

end

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [data] = initControlSystem(parameters,data)
load('DesignProblem04_EOMs.mat');
syms phi phidot v w tauR tauL elateral eheading vroad wroad
f = symEOM.f;

fsym=[phidot;f(1);f(2);f(3); -v*sin(eheading); w-((v*cos(eheading))/(vroad+wroad*elateral))*wroad];

syms V W TAUR TAUL real
%Equilibrium Points
[phie, phidote, ve, we, elaterale, eheadinge, tauLe, tauRe, vroade, wroade] = DP4_fsolve;
xhat = [0; 0; -ve; 0; 0; 0];
eqstate = [phie; phidote; ve; we; elaterale;eheadinge];
eqinput = [tauLe; tauRe];
state = [phi; phidot; v; w; elateral; eheading];
input = [tauL; tauR];
%Linearization

A = double(subs(jacobian(fsym,state),[state; input; vroad; wroad],[eqstate;eqinput;vroade;wroade]));
B = double(subs(jacobian(fsym,input),[state; input; vroad; wroad],[eqstate;eqinput;vroade;wroade]));
C = [0 0 (1/parameters.r) ((parameters.b)/(2*parameters.r)) 0 0;
    0 0 (1/parameters.r) (-(parameters.b)/(2*parameters.r))  0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;];


controllability = rank(ctrb(A,B))-length(A);
observability = rank(obsv(A,C)) -length(A);



Qc = 1.5*[1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 30000 0;0 0 0 0 0 2750];
Rc = [1 0;0 1];
K = lqr(A,B,Qc,Rc)
%Verifying stability 
eig(A-B*K);

%Define Gains Observer
Ro = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];
Qo = [1 0 0 0 ;0 1 0 0 ;0 0 1 0 ;0 0 0 1];
L = lqr(A',C',inv(Ro),inv(Qo))'

%Verifying stability 
eig(A-L*C);


%Saving Variables
data.A = A;
data.B = B;
data.C = C;
data.h = parameters.tStep;
data.xhat = xhat;
data.eqstate = eqstate;
data.eqinput = eqinput;
data.K = K;
data.L = L;
end

%
% Here is a good place to initialize things like gains...
%
%   data.K = [1 2 3 4];
%
% ...or like the integral of error in reference tracking...
%
%   data.v = 0;
%


%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators,data] = runControlSystem(sensors,references,parameters,data)


A = data.A;
B = data.B;
C = data.C;
eqstate = data.eqstate;
eqinput = data.eqinput;
h = parameters.tStep;
xhat = data.xhat;


output = [sensors.wR; sensors.wL; sensors.e_lateral;sensors.e_heading];
K = data.K;
L = data.L;

y = output - C*[eqstate];%error
u =-K*xhat;%input
dxhat = A*xhat+B*u-L*(C*xhat-y);%this is obersever. State Estimate
data.xhat = xhat + h*dxhat; % Next step for state estimate.


actuators.tauR = u(2) + eqstate(2) + 0.12;
actuators.tauL = u(1) + eqinput(1) + 0.12;
end

 function [phie, phidote, ve, we, elaterale, eheadinge, tauLe, tauRe, vroade, wroade] = DP4_fsolve

syms phi phidot v w tauL tauR elateral eheading vroad wroad
load('DesignProblem04_EOMs.mat');
f = symEOM.f;

x = [phi; phidot; v; w; elateral; eheading; tauL; tauR; vroad; wroad];
f_numeric = matlabFunction(f, 'vars', {x});

x_guess = [0.25; 0; 0.5; 0; 0; 0; 0; 0; 5; 0];

f_numeric_at_x_guess = f_numeric(x_guess);

options = optimoptions(...
    'fsolve', ...                           % name of solver
    'algorithm', 'levenberg-marquardt', ... % algorithm to use
    'functiontolerance', 1e-12, ...         % smaller means higher accuracy
    'steptolerance', 1e-12);                % smaller means higher accuracy
[xe, f_numeric_at_x_sol, exitflag] = fsolve(f_numeric, x_guess, options);

 % Parse solution
 phie = xe(1);
 phidote = xe(2); 
 ve = xe(3);
 we = xe(4);
 elaterale = xe(5);
 eheadinge = xe(6);
 tauLe = xe(7);
 tauRe = xe(8);
 vroade = xe(9);
 wroade = xe(10);
 end