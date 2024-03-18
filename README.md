# ME405-Term-Project

OVERVIEW

This project's goal was to create a Nerf gun turret that uses a thermal camera and motors to aim at and shoot a stationary target at a set distance away. 
It was intended as a competition between other teams in the same Mechatronics class section as whoever could shoot faster and hit more reliably would win the tournament-style competition.

HARDWARE

Our team, the Scorching Scallywags, designed our system to aim with a Rev Robotics motor and pull the trigger with a servo. 

Laser-cut acrylic is used for the pulleys and the base. 
3D printed parts are used to mount the motor, servo, and Nerf gun. 

A Python differential equation simulation was used to find that the best pulley ratio (fastest and most stable) was 1:1 since the motor already has a 72:1 gear ratio built in.

Our motor, servo, and thermal camera were controlled and processed with a STM32L476 Nucleo microcontroller. 
The microcontroller was powered by a standard power supply set to 12V. 
A voltage regulator circuit reduced the voltage to 5V for the servo to use.

CAD Model:

![image](https://github.com/JoshuaTuttobene/ME405-Term-Project/assets/107731390/c1119716-5263-4318-9854-b6561a143145)

Hardware Picture (REPLACE WITH HIGHER QUALITY IMAGE):

![image](https://github.com/JoshuaTuttobene/ME405-Term-Project/assets/107731390/130b673a-414e-4084-8b53-82ca18a79b9a)


SOFTWARE

The program calculates a thermal centroid from a processed image, aims the gun at that centroid with a PD controller, and then pulls the trigger once locked onto the target.

In depth documentation can be found here: INSERT DOXYGEN MAIN PAGE LINK

RESULTS

Our resultant turret performed surprisingly well for a project designed, built, programmed, and tuned in just a few weeks. 
Our team tied for first in our section's tournament, but lost in sudden death.

We tested our system thoroughly by simulating competition conditions as we became able to. 

First, we tried rotating the gun 180 degrees. Then, we added PD control. We tried PID control, but that ended up being highly unstable.
Once we could aim at a setpoint, we worked on the thermal camera's image processing to obtain the desired setpoint. 
Then, we tested the trigger mechanism. Once we had the mechanics of it figured out, it was simple to program.

We tested and tuned the aiming by comparing the desired setpoint from the thermal camera to the actual location of the target and how close the gun came to aiming at that setpoint. 
This took a lot of iteration. We had to adjust the camera's location to get a more precise image and adjust our trigonometry for the centroid whenever doing so. 
We also had to add in an offset to the desired setpoint into the code as the mechanical system would consistently stray to one side of the desired setpoint.

We eventually noticed that a lot of our inconsistency in aiming came from the backlash in the motor itself. 
We treated this issue by always setting up the gun in the same exact way to minimize the effect of backlash on the aiming.
Unfortunately, the backlash still played a role in inconsistency.
The built in encoder also had a resolution that played a significant role in our consistency. 
Each encoder tick corresponded to 1.25 degrees giving our accuracy an inherent +-0.625 degree error since we couldn't figure out how drive the motor to a setpoint in between encoder values if such a thing is even possible.

POSSIBLE IMPROVEMENTS

In retrospect, we should've designed a more robust mechanical system. Using a stepper motor would have nearly eliminated our backlash problem. 
A motor with a high enough step count would also result in a better aiming resolution than our encoder could do.
We could also have used a larger gear ratio in order to get more precise aiming. This would sacrifice some of our speed though.
Better tensioning on our timing belt or using precisely meshing gears would also have helped our turret be more consistent.

Our software was also not blameless. Our image processing took much longer than we expected. Even after optimizing it the best we could and running the microcontroller at its maximum frequency, it still took around 3 seconds to take an image and spit out a setpoint. 
On top of that, simply creating an instance (object) of the camera class took over a second. Since our mechanical system could turn 180 degrees in a quarter of a second, the camera ended up being the slowest component by far.

Overall, however, we are quite pleased with our performance and happy that we learned a lot about how to better design and optimize mechatronic systems.
