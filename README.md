# CubeSATSim Project
The goal of this project is to simulate and test the attitude control of a 1U CubeSAT. In the end, the CubeSAT will be given fake sensor data from the 3D simulation and the reaction of the attitude control system will be read and fed into the simulation.

# Setting
The CubeSAT's orbit will have an altitude of 400km - 1000km. It will be equipped with a variety of sensors, including a three-axis gyroscope sensor, a three-axis magnetometer and a (maybe selfbuild) horizon sensor. The only way it will be able to control its attitude  is by using the three magnetorquers it is equipped with.

# The 3D Simulation

The 3D Simulation is written in Matlab and considers the following effects.
- the gravitational force of the earth causing the CubeSAT to orbit. Only the raw force to the satellite's center of mass is regarded, the torque created by a specific distribution of mass in the satellite is ignored at the moment. This could be implemented at a later point of time when the detailed construction plan is known.
- the force and torque of the earth's magnetic field on the the magnetorquers.

## The Earth's magnetic field
The magnetic field is regarded as being created by a magnetic dipole. The direction of the magnetic dipole moment is assumed to be the same as the rotation axis and the strength is (according to own estimations) set to `10^23  A*m²`. The magnetic field created by such a magnetic dipole moment looks like this (By Bdushaw (Own work) [CC BY-SA 4.0 (http://creativecommons.org/licenses/by-sa/4.0)], via Wikimedia Commons):  
![2D magnetic field](https://upload.wikimedia.org/wikipedia/commons/5/51/Magnetic_dipole_moment.jpg "2D magnetic field")  
The resulting 3D magnetic field looks like this:
![3D magnetic field](https://raw.githubusercontent.com/ff17x3/CubeSATSim/master/images/BField.png "3D magnetic field")  
The magnetic flux density vector is calculated by using the the formula from [this reference]( https://en.wikipedia.org/wiki/Magnetic_dipole#External_magnetic_field_produced_by_a_magnetic_dipole_moment).
Thus far it is not finally clear if this model is accurate enough and especially if the "horizontal scale" of the field matches the earth's one.

# The Control Algorithm

## Changing attitude
The only way to change the satellite’s attitude is by powering the magnetorquers. These can create a torque resulting in an angular acceleration. The problem however is that the torque is always perpendicular to the magnetic flux density vector (the formula is `t = m x B`). So the satellite cannot be rotated arbitrary. In fact, if the satellite is rotating around the axis parallel to the magnetic field vector the magnetorquers cannot not change that at all. Luckily the CubeSAT is orbiting the earth with an inclination. While the amount of inclination is not known yet it is reasonable to expect the inclination be not greater than zero. So while the satellite may rotate around the axis parallel to the magnetic field vector at one point, due to the inclination the orientation of the magnetic field vector changes as the CubeSAT moves further on its orbit. Therefore it is always possible to change the rotation velocity according to any given axis, at least in theory.


## Part I: Detumbling
In the _detumbling_ part of the attitude control algorithm the goal is the stop any rotation thereby stabilize the attitude of the satellite.  

The first approach arises from the knowledge that a higher





