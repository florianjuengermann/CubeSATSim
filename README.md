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
Thus far it is not finally clear if this model is accurate enough and especially if the "horizontal scale" of the field matches the earth's one.

# The Control Algorithm



