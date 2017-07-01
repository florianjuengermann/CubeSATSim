clear variables;

%--------- GLOBAL CONSTANTS----------------
global MU_0 GAMMA EARTH_RADIUS EARTH_MASS T;
EARTH_RADIUS = 6371000;
EARTH_MASS = 5.972e24;
GAMMA = 6.674e-11;
MU_0 = pi*4e-7;
T = 1;

% $x^2+e^{\pi i}$
%-------- CUBESAT PARAMETERS--------------
global HEIGHT J CUBE_MASS;
HEIGHT = 400000;
J = [ [1.0/600, 0, 0]; [ 0, 1.0/600, 0]; [0, 0, 1.0/600]]; 
CUBE_MASS = 1;



V0 = sqrt(GAMMA * EARTH_MASS / EARTH_RADIUS)

dipoleEarth = [0; 0; 1e23];
dipoleCube = [1; 0; 0]; %TODO test only

posSAT = [EARTH_RADIUS+HEIGHT; 0; 0]; 
veloSAT = [0; V0; 0];

angularVel = [0; 0; 0];
dirSAT = [0; 0; 0];

B = mFluxDesity(posSAT, dipoleEarth);
F_G = gravityEarth(posSAT, 1)
F_m = magneticForce(posSAT, dipoleCube, dipoleEarth)

% Plotting

figure
hold on
toPlot = [];
x=1:1:10000;

for i = x
    
    %------- CUBESAT POSITION -------
    F_G = gravityEarth(posSAT, 1);
    % TODO Calculate dipoleCube based on time-varying input
    F_m = magneticForce(posSAT, dipoleCube, dipoleEarth);
    
    accSAT = (F_G + F_m) / CUBE_MASS;
    
    veloSAT = veloSAT + accSAT * T;
    posSAT = posSAT + veloSAT * T;
    
    
    
    %------- CUBESAT ATTITUDE -------
    B = mFluxDesity(posSAT, dipoleEarth);
    tSAT = magneticTorque(B, dipoleCube)
    
    angularAcc =  J \ tSAT; % inv(J) * tSAT;
    angularVel = angularVel + angularAcc * T;
    dirSAT = rotateVec(angularVel / norm(angularVel), dirSAT, norm(angularVel));

    toPlot = [toPlot dirSAT(2,1)*1e4];
end

plot(x, toPlot)

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
%   m: magnetic dipole moment
    t = cross(m, B);
end

function vRot = rotateVec(k, v, theta)
%   k: rotation axis (unit vector)
%   v: vector to be rotated around k
%   theta: rotation angle (in radians)
    vRot = v * cos(theta) + cross(k,v) * sin(theta) + k * dot(k,v) * (1 - cos(theta));
end