clear variables;
% linux 5 6
%--------- GLOBAL CONSTANTS----------------
global MU_0 GAMMA EARTH_RADIUS EARTH_MASS DIPOLE_EARTH;
EARTH_RADIUS = 6371000;
EARTH_MASS = 5.972e24;
GAMMA = 6.674e-11;
MU_0 = pi*4e-7;
DIPOLE_EARTH = [0; 0; 1e23];

%--------- SIMULATION PARAMETERS------------
global SIM_TIME DRAW_STEPS T CALC_STEPS SIM_FACTOR;
SIM_TIME = 5545*0.2;%zoomed out (whole circle) ~5000 seconds 
DRAW_STEPS = 200;
T = 0.5*0.5;

CALC_STEPS = SIM_TIME / T;
SIM_FACTOR = 1.0 * CALC_STEPS / DRAW_STEPS;



%-------- CUBESAT PARAMETERS--------------
global HEIGHT J CUBE_MASS COIL_WHORLS MU COIL_LENGTH COIL_CROSSAREA COIL_RESISTANCE COIL_INDUCTANCE;
HEIGHT = 400000;
J = [ [1.0/600, 0, 0]; [ 0, 1.0/600, 0]; [0, 0, 1.0/600]]; 
CUBE_MASS = 1;
MU = 0.006; % depends on core material
% Magnetorquers
COIL_WHORLS = 500;
COIL_LENGTH = 0.05; % 5cm
COIL_CROSSAREA = 0.000001; % 1cm x 1cm
COIL_RESISTANCE = 30;
COIL_INDUCTANCE = COIL_CROSSAREA * COIL_WHORLS^2 * MU / COIL_LENGTH; % MU_R = MU / MU_0




%-------- ATTITUDE CONTROL PARAMETERS--------------
global ENERGY_SAVE_MODE PROPORTIONAL_COEFF DETUMBLING CENTER_LAST_COUNT PROPORTIONAL_COEFF_ACTIVE;
CENTER_LAST_COUNT = 50;
PROPORTIONAL_COEFF = 3e3;
PROPORTIONAL_COEFF_ACTIVE = 5e3;
ENERGY_SAVE_MODE = true;
DETUMBLING = false;
V0 = sqrt(GAMMA * EARTH_MASS / (EARTH_RADIUS+HEIGHT));

I = [0;0;0];%dir: dirSat; dirNorm;  cross(dirSat,dirNorm)
U = [0;0;0];
energy = 0;

posSAT = [EARTH_RADIUS+HEIGHT; 0; 0]; 
inclAngle = 0.0;
veloSAT = [0; cos(inclAngle)*V0; sin(inclAngle)*V0];


angularVel = [0; 0; 0];% global coordinate system
angularVelRel = [0; 0; 0]; %relative to Cubsat -> gyroscope values

%{
B = mFluxDesity(posSAT, dipoleEarth);
F_G = gravityEarth(posSAT, 1)
F_m = magneticForce(posSAT, dipoleCube, dipoleEarth)
%}

dirSAT = [-1; 0; 0]; % Camera perspective
dirNormalSAT = [0; -1; 0]; % Normal vector to diSAT, pointing to a specific face
dipoleCube = dirSAT*0; %TODO test only

toCenterVec = [0; 0; 0]; % Vector showing dirSAT's derivation from direction to earth's center 
toCenterLast = zeros(3, CENTER_LAST_COUNT);


% Plotting
toPlotDir = zeros(3,DRAW_STEPS);
toPlotDirN = zeros(3,DRAW_STEPS);
toPlotPos = zeros(3,DRAW_STEPS);
toPlotComp = zeros(3,DRAW_STEPS);
toPlotVelo = zeros(4,DRAW_STEPS);
toPlotI = zeros(3, DRAW_STEPS);
toPlotU = zeros(3, DRAW_STEPS);
toPlotW = zeros(1, DRAW_STEPS);
toPlotMreq = zeros(3, DRAW_STEPS);
toPlotAttDif = zeros(2,DRAW_STEPS);
toPlotVeloC = zeros(3,DRAW_STEPS);
plotTime = zeros(1,DRAW_STEPS);

a = 0;

x=1:1:CALC_STEPS;

for i = x
    
    %------- COILS / MAGNETORQUERS -------
    I = currentChange(I, U, T);
    energy = energy + (abs(I(1) * U(1)) + abs(I(2) * U(2)) + abs(I(3) * U(3))) * T;

    %------- CUBESAT POSITION -------
    F_G = gravityEarth(posSAT, 1);
    % TODO Calculate dipoleCube based on time-varying input
    F_m = magneticForce(posSAT, dipoleCube, DIPOLE_EARTH); %TODO test only
    
    accSAT = (F_G + F_m) / CUBE_MASS;
    
    veloSAT = veloSAT + accSAT * T;
    posSAT = posSAT + veloSAT * T;
    
    
    
    %------- CUBESAT ATTITUDE -------
    tSAT = magneticTorqueSAT(posSAT, dirSAT, dirNormalSAT, I);
    
    angularAcc =  J \ tSAT; % inv(J) * tSAT;
    angularVel = angularVel + angularAcc * T;
    angularRotChange = angularVel * T;
    
    if(norm(angularRotChange) ~= 0)
        dirSAT = rotateVec(angularRotChange / norm(angularRotChange), dirSAT, norm(angularRotChange));
        dirNormalSAT = rotateVec(angularRotChange / norm(angularRotChange), dirNormalSAT, norm(angularRotChange));
    end
    
    angularVelRel = getRelVec(angularVel, dirSAT, dirNormalSAT);
    Brel = getRelVec(mFluxDesity(posSAT, DIPOLE_EARTH), dirSAT, dirNormalSAT);
    mRequiredRel = [0,0,0];
    rotAngle = 100;
    targetCF = [0;0;0];

    if (DETUMBLING)
        if(false)
            %------- ATTITUDE CONTROL ATTEMPT 1 -------
            %------- Phase 1: Detumbling -------
            if(norm(angularVelRel) ~= 0)
                mRequiredRel = getEstimatedDipoleMomentum(Brel, -angularVelRel, J);
            end
            
        else 
            %------- ATTITUDE CONTROL ATTEMPT 2   
            rotB = Brel * dot(angularVelRel, Brel) / norm(Brel)^2; %rotVel projected onto B
            rotNotB = rotB - angularVelRel; %other part of the rotVel, normal to rotB
            if(norm(rotNotB) ~= 0)
                t_target = rotNotB / norm(rotNotB);
                mRequiredRel = cross(-t_target, Brel) * norm(rotNotB) * PROPORTIONAL_COEFF;
            end
        end
    else
        %----------ACTIVE CONTROL---------------------
        target = [1; 0; 0]; %in sunray direction (AF)
        targetCF = AFToCF(target, getSunray(dirSAT, dirNormalSAT), Brel);
        distAngle = acos(dot([1;0;0],targetCF) / norm(targetCF)); %x-dir
        rotAxis = getRotAxis(targetCF, Brel);
        if(norm(rotAxis)~=0)
            rotAngle = getRotAngle(targetCF, rotAxis);
            
            if(i*T/5545.0 > 0.019)
                'there';
            end
            targetVelo = rotAxis * abs(distAngle)^1 * 3e-2;
            veloChange = targetVelo - angularVelRel;
            mRequiredRel = cross(veloChange, Brel)/ norm(cross(veloChange, Brel)) * norm(veloChange)^2 *-3e1 * PROPORTIONAL_COEFF_ACTIVE;
            if(norm(mRequiredRel) > 1)
                mRequiredRel = mRequiredRel/norm(mRequiredRel);
            end
        end
    end

    if(norm(mRequiredRel) ~= 0)
        targetI = solenoidNeededCurrent(mRequiredRel);

        if(ENERGY_SAVE_MODE)
           % just set the current which will asymptotically lead to the right current
           U = getVoltageByTargetCurrent(targetI);
        else
           % TODO 
        end
    end
    

    %-----DRAWING-----------
    if ( floor(i / SIM_FACTOR) >  floor((i-1)/SIM_FACTOR))
        toPlotDir(:, floor(i/SIM_FACTOR)) = dirSAT*5e5;
        toPlotDirN(:, floor(i/SIM_FACTOR)) = dirNormalSAT*5e5;
        toPlotPos(:, floor(i/SIM_FACTOR)) = posSAT;
        attDif = -targetCF + [1;0;0];
        toPlotAttDif(:, floor(i/SIM_FACTOR)) = [rotAngle; distAngle];
        toPlotVeloC(:, floor(i/SIM_FACTOR)) = veloChange;
        toPlotVelo(:, floor(i/SIM_FACTOR)) = [(angularVel(1)); (angularVel(2)); (angularVel(3));0];
        toPlotI(:, floor(i/SIM_FACTOR)) = I;
        toPlotU(:, floor(i/SIM_FACTOR)) = U;
        toPlotW(:, floor(i/SIM_FACTOR)) = energy;
        toPlotMreq(:, floor(i/SIM_FACTOR)) = mRequiredRel;
        plotTime( floor(i/SIM_FACTOR)) = i*T/5545.0;
        B = mFluxDesity(posSAT, DIPOLE_EARTH);
        toPlotComp(:,floor(i/SIM_FACTOR)) = [getUsablity(B,dirSAT); getUsablity(B,dirNormalSAT); getUsablity(B, cross(dirSAT, dirNormalSAT))];
    end
end

%draw B field
maxDist = EARTH_RADIUS-2*HEIGHT;
Bresolution = 2;
values = -maxDist:maxDist/Bresolution:maxDist;
Coords = zeros(3,length(values)*length(values)*length(values));
bStrength = zeros(3,length(values)*length(values)*length(values));
c = 1;
for i = values
    for j = values
        for k = values
            BVec =  mFluxDesity([i;j;k],DIPOLE_EARTH);
            BVec = BVec / norm(BVec);
            Coo = [i,j,k];
            for n = 1:3
                Coords(n,c) = Coo(n);
                bStrength(n,c) = BVec(n);
            end
            c = c + 1;
        end
    end
end


%{
figure
hold on
quiver3(Coords(1,:),Coords(2,:),Coords(3,:),bStrength(1,:),bStrength(2,:),bStrength(3,:),'AutoScale','on');

quiver3(toPlotPos(1,:),toPlotPos(2,:),toPlotPos(3,:),toPlotDir(1,:),toPlotDir(2,:),toPlotDir(3,:),'AutoScale','on');
quiver3(toPlotPos(1,:),toPlotPos(2,:),toPlotPos(3,:),toPlotDirN(1,:),toPlotDirN(2,:),toPlotDirN(3,:),'AutoScale','on');
axis equal;
view(0,90);
%}

%{
% Detumbling Comparison
load('data/veloAttempt1_28deg_1rad_s.mat');
figure 
plot(plotTime, vertcat(SavedVelo1(1,:), toPlotVelo(1,:)));
set(gca, 'YScale', 'log');
xlabel('time [revoltutions]');
ylabel('ang.Velo. [rad/s]');
legend('Attempt 1','Attempt 2');
%}

figure 
plot(plotTime,toPlotAttDif);
xlabel('time [revoltutions]');
ylabel('attitude derivation [rad]');
legend('rot axis','direct');

figure
plot(plotTime,toPlotVeloC);
xlabel('time [revoltutions]');
ylabel('veloChange');

figure
plot(plotTime,toPlotVelo);
xlabel('time [revoltutions]');
ylabel('velo');



%}
%{
figure
plot(plotTime,toPlotW);
xlabel('time [revoltutions]');
ylabel('Power used [Ws]');

figure
plot(plotTime,toPlotU);
xlabel('time [revoltutions]');
ylabel('Voltage [V]');
%}
%{
figure

plot(plotTime,toPlotMreq);
xlabel('time [revoltutions]');
ylabel('Mreq [I]');
%}
%{
plot(toPlotComp(1,:));
plot(toPlotComp(2,:));
plot(toPlotComp(3,:));
%plot(toPlotComp(1,:) + toPlotComp(2,:)  + toPlotComp(3,:));
%}



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

function s = getSunray(cubedir, cubenorm)
    s = getRelVec([0; -1;0],cubedir, cubenorm);
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
    global MU_0 MU COIL_WHORLS
    % MU_R = MU / MU_0
    m = COIL_WHORLS * I * A * MU / MU_0;
end

function I = solenoidNeededCurrent(m)
%   m: magnitude of dipole momentum
    global MU_0 MU COIL_WHORLS COIL_CROSSAREA
    I = m * MU_0 / (MU * COIL_WHORLS * COIL_CROSSAREA);
end

function t = magneticTorqueSAT(posSAT, dirSAT, dirNormalSAT, I)
%   I: current flowing trough coil
%   A: cross sectional area (normal vector)
    global DIPOLE_EARTH COIL_CROSSAREA;
    magnetorquer1 = solenoidDipoleMomentum(I(1), ( dirSAT / norm(dirSAT) ) * COIL_CROSSAREA);
    magnetorquer2 = solenoidDipoleMomentum(I(2), ( dirNormalSAT / norm(dirNormalSAT) ) * COIL_CROSSAREA);
    magnetorquer3 = solenoidDipoleMomentum(I(3), ( cross(dirSAT, dirNormalSAT) / norm(cross(dirSAT, dirNormalSAT)) ) * COIL_CROSSAREA);

    BSAT = mFluxDesity(posSAT, DIPOLE_EARTH);

    t = magneticTorque(BSAT, magnetorquer1) + magneticTorque(BSAT, magnetorquer2) + magneticTorque(BSAT, magnetorquer3);
end

function c = getUsablity(v, u)
%   v: Vector which should be composed
%   u: Vector which describes the axis

    c =  1-(abs(getComponent(v, u)/norm(v)));
end

function c = getComponent(v, u)
%   v: Vector which should be composed
%   u: Vector which describes the axis

    c =  dot(v,u) / norm(u);
end
function v = getRelVec(in, a, b)
%   in: vector which should be composed
%   a, b: the vectors which describe the axis of the relative coordinate
%   system
    v = [getComponent(in, a);
         getComponent(in, b);
         getComponent(in, cross(a, b))];
end
function m = getEstimatedDipoleMomentum(B, targetTorque, J)
%   B: magnetic flux density 
%   targetTorque: axis and amound of torque desired
%   J: moment of inertia

%   anglPerc: 1.0 -> B and targetTorque are perpendicular
%             0.0 -> B and targetTorque are parallel
    global PROPORTIONAL_COEFF;
    anglPerc = abs(acos(dot(B, targetTorque)/norm(B) / norm(targetTorque))-0.5*pi)/(0.5*pi);
    A = J * targetTorque * norm(targetTorque)^0.5 * 0.04 * (1.01 - anglPerc)^1.1; 
    if(norm(cross(B, A)) ~= 0)
        m = ( cross(B, A) / norm(cross(B, A)) ) * norm(A) / norm(B)  * 3e-4 * PROPORTIONAL_COEFF;
    else
        m = [0,0,0];
    end
end

function v = getVoltageByTargetCurrent(I)
%   I: target current of the magnetorquers
    global COIL_RESISTANCE;
    v = COIL_RESISTANCE * I;
end

function I_new = currentChange(I, U, dt)
%   I: current current of the coil
%   U: current voltage at the coil
%   dt: time past since last update

    global COIL_RESISTANCE COIL_INDUCTANCE;
    I_new  = U / COIL_RESISTANCE - (U / COIL_RESISTANCE - I) * exp( - dt * COIL_RESISTANCE / COIL_INDUCTANCE);
end

% Attitude Frame vector to Cubesat Frame vector
function u = AFToCF(v, s, B)
%   v:      vector in Attitude Frame
%   s:    sunray vector
%   B:      magnetic field vector
%   return: v in Cubsat Frame
    
    % b (to s) the perpendicular part of B
    b = B - dot(B, s) / norm(s) * s / norm(s);
    u = v(1)*s + v(2)*b + v(3)*cross(s,b);
    %u = dot(v, s)/norm(s) * [1;0;0] + dot(v, b)/norm(b) * [0;1;0] + dot(v, cross(v, b))/norm(cross(v, b)) * [0;0;1];

end

function rot = getRotAxis(u, B)
%   u:      target attitude
%   B:      magnetic field vector
%   return: best rot. axis to move [1;0;0] to u
    
    % v: current attitude
    v = [1; 0; 0];
    % w: normal vector of plane of all possible rotation vectors
    w = cross(v + u, cross(v, u));

    if (norm(w) == 0)
        %u and v are colinear (could be opposite)
        r = [0;0;0];
    elseif (B(3)*w(2) - B(2)*w(3) == 0 || B(3) == 0)
        r = [0;0;0];
    else
        r = [1;
            (B(1)*w(3) - B(3)*w(1))/(B(3)*w(2) - B(2)*w(3));
            -B(1)/B(3) - B(2) * (B(1)*w(3) - B(3)*w(1))/(B(3) * (B(3)*w(2) - B(2)*w(3)) )];
        r = r / norm(r);
    end 
    rot = r;
end

function alpha = getRotAngle(u,r)
%   u:      target attitude 
%   r:      rotation axis
%   return: angle needed to map [1;0;0] to u by a rotation around r

    % v: current attitude
    v = [1; 0; 0];

    f = r * dot(r, u)/norm(r)^2;
    alpha = acos(dot(u-f, v-f) / (norm(u-f)*norm(v-f)) );
end

