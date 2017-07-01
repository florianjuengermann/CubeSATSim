clear variables;

%--------- GLOBAL CONSTANTS----------------
global MU_0 GAMMA EARTH_RADIUS EARTH_MASS T DIPOLE_EARTH STEPS SIM_FACTOR;
EARTH_RADIUS = 6371000;
EARTH_MASS = 5.972e24;
GAMMA = 6.674e-11;
MU_0 = pi*4e-7;
T = 0.05;
STEPS = 50;
SIM_FACTOR = 700;
DIPOLE_EARTH = [0; 0; 1e23];

% $x^2+e^{\pi i}$
%-------- CUBESAT PARAMETERS--------------
global HEIGHT J CUBE_MASS COIL_TURNS MU COIL_CROSSAREA;
HEIGHT = 400000;
J = [ [1.0/600, 0, 0]; [ 0, 1.0/600, 0]; [0, 0, 1.0/600]]; 
CUBE_MASS = 1;
% Magnetorquers
COIL_TURNS = 500;
COIL_CROSSAREA = 0.000001;
MU = 1;


V0 = sqrt(GAMMA * EARTH_MASS / EARTH_RADIUS);


I_1 = 1;
I_2 = 1;
I_3 = 1;

posSAT = [EARTH_RADIUS+HEIGHT; 0; 0]; 
veloSAT = [0; V0; 0];

angularVel = [0; 0; 0];

%{
B = mFluxDesity(posSAT, dipoleEarth);
F_G = gravityEarth(posSAT, 1)
F_m = magneticForce(posSAT, dipoleCube, dipoleEarth)
%}

dirSAT = [1; 0; 0];
dirNormalSAT = [0; 1; 0]; % Normal vector to diSAT, pointing to a specific face
dipoleCube = dirSAT*1; %TODO test only


% Plotting

figure
hold on
toPlotDir = zeros(3,STEPS);
toPlotDirN = zeros(3,STEPS);
toPlotPos = zeros(3,STEPS);
x=1:1:STEPS*SIM_FACTOR;

for i = x
    
    %------- CUBESAT POSITION -------
    F_G = gravityEarth(posSAT, 1);
    % TODO Calculate dipoleCube based on time-varying input
    F_m = magneticForce(posSAT, dipoleCube, DIPOLE_EARTH); %TODO test only
    
    accSAT = (F_G + F_m) / CUBE_MASS;
    
    veloSAT = veloSAT + accSAT * T;
    posSAT = posSAT + veloSAT * T;
    
    
    
    %------- CUBESAT ATTITUDE -------
    B = mFluxDesity(posSAT, DIPOLE_EARTH);
    tSAT = magneticTorqueSAT(posSAT, dirSAT, dirNormalSAT, I_1, I_2, I_3)   % TODO Change I_x over time
    
    angularAcc =  J \ tSAT; % inv(J) * tSAT;
    angularVel = angularVel + angularAcc * T;
    
    if(norm(angularVel) ~= 0)
        dirSAT = rotateVec(angularVel / norm(angularVel), dirSAT, norm(angularVel));
        dirNormalSAT = rotateVec(angularVel / norm(angularVel), dirNormalSAT, norm(angularVel));
    end
    if (mod(i, SIM_FACTOR) == 0)
        toPlotDir(:,i) = dirSAT*5e5;
        toPlotDirN(:,i) = dirNormalSAT*5e5;
        toPlotPos(:,i) = posSAT;
    end
    
       

end

quiver3(toPlotPos(1,:),toPlotPos(2,:),toPlotPos(3,:),toPlotDir(1,:),toPlotDir(2,:),toPlotDir(3,:),'AutoScale','off');
quiver3(toPlotPos(1,:),toPlotPos(2,:),toPlotPos(3,:),toPlotDirN(1,:),toPlotDirN(2,:),toPlotDirN(3,:),'AutoScale','off');
axis equal;
view(0,90);
%feather(toPlotDirX,toPlotDirY);

%plot(x, toPlot)

function F_G = gravityEarth(r, m)
%   r: from earth's center to location
%   m: mass of object
    global EARTH_MASS GAMMA;
    F_G = GAMMA * EARTH_MASS * m / (norm(r)^2) * r / (-norm(r));

end


function B = mFluxDesity(r, m )
%   r: from magentic dipole m to location
%   m: magnetic dipole momentum (Vector)
    global MU_0;
    B = MU_0* 1 / ( 4 * pi) * ( (3*r*dot(r,m)) / (norm(r)^5)  - m / (norm(r)^3) );

end

function F_m = magneticForce(r, m, mE)
%   m2 : m   ,   m1: mE
%   r: from earth's center to location
%   m: magnetic dipole momentum of object
%   mE: magnetic dipole momentum of the earth
    global MU_0;
    rh = r / norm(r);
    F_m = 3 * MU_0 /( 4* pi * (norm(r)^4)) * ( m * dot(mE, rh) + mE * dot(m,r) + rh * dot(m, mE) - 5 * rh * dot(mE, rh) * dot(m, rh));
end

function t = magneticTorque(B, m)
%   B: magnetic flux density
%   m: magnetic dipole momentum
    t = cross(m, B);
end

function vRot = rotateVec(k, v, theta)
%   k: rotation axis (unit vector)
%   v: vector to be rotated around k
%   theta: rotation angle (in radians)
    vRot = v * cos(theta) + cross(k,v) * sin(theta) + k * dot(k,v) * (1 - cos(theta));
end

function m = solenoidDipoleMomentum(I, A)
%   I: current flowing trough coil
%   A: cross sectional area (normal vector)
    global MU_0 MU COIL_TURNS
    m = COIL_TURNS * I * A * MU / MU_0;
end

function t = magneticTorqueSAT(posSAT, dirSAT, dirNormalSAT, I_1, I_2, I_3)
%   I: current flowing trough coil
%   A: cross sectional area (normal vector)
    global DIPOLE_EARTH COIL_CROSSAREA
    magnetorquer1 = solenoidDipoleMomentum(I_1, ( dirSAT / norm(dirSAT) ) * COIL_CROSSAREA);
    magnetorquer2 = solenoidDipoleMomentum(I_2, ( dirNormalSAT / norm(dirNormalSAT) ) * COIL_CROSSAREA);
    magnetorquer3 = solenoidDipoleMomentum(I_3, ( cross(dirSAT, dirNormalSAT) / norm(cross(dirSAT, dirNormalSAT)) ) * COIL_CROSSAREA);
    
    BSAT = mFluxDesity(posSAT, DIPOLE_EARTH);

    t = magneticTorque(BSAT, magnetorquer1) + magneticTorque(BSAT, magnetorquer2) + magneticTorque(BSAT, magnetorquer3);
end