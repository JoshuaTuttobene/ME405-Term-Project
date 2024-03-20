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

Hardware Picture:

![image](https://github.com/JoshuaTuttobene/ME405-Term-Project/assets/107731390/dd470a79-701d-41e5-9cae-7b37eccfa437)

Circuit Diagram:

![image](https://github.com/JoshuaTuttobene/ME405-Term-Project/assets/107731390/02fcccbe-8262-48c4-854e-00795db5db9c)



SOFTWARE

The program calculates a thermal centroid from a processed image, aims the gun at that centroid with a PD controller, and then pulls the trigger once locked onto the target.

In depth documentation can be found here: https://joshuatuttobene.github.io/ME405-Term-Project/

Task Diagram:

![image](https://github.com/JoshuaTuttobene/ME405-Term-Project/assets/107731390/385fb0bc-023f-4a9d-9e61-1c0eb9aec867)

Finite State Machine:

![image](https://github.com/JoshuaTuttobene/ME405-Term-Project/assets/107731390/8ee23383-57e8-4b4d-80e2-7741f3c7965a)

RESULTS

Our resultant turret performed surprisingly well for a project designed, built, programmed, and tuned in just a few weeks, especially considering that we were balancing other schoolwork, labs, and projects at the same time.
Our team tied for first (out of 6 teams) in our section's tournament, but lost in sudden death.

We tested our system thoroughly by simulating competition conditions as we became able to. 

First, we tried rotating the gun 180 degrees. Then, we added PD control. We tried PID control, but that ended up being highly unstable and the PD control worked just fine, so we scrapped the integral.
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

In retrospect, we could've designed a more robust mechanical system. Using a stepper motor would have nearly eliminated our backlash problem. 
A motor with a high enough step count would also result in a better aiming resolution than our encoder could do.
We could also have used a larger gear ratio in order to get more precise aiming. This would sacrifice some of our speed though.
Better tensioning on our timing belt or using precisely meshing gears would also have helped our turret be more consistent.
We also slightly misjudged the angle on our gun mount, which made it aim for headshots. Fortunately, we were able to shim some scrap metal underneath our mount to make it aim at chest-level for the competition since the chest is easier to hit.

Our software was also not blameless. Our image processing took much longer than we expected. Even after optimizing it the best we could and running the microcontroller at its maximum frequency, it still took around 3 seconds to take an image and spit out a setpoint. 
On top of that, simply creating an instance (object) of the camera class took over a second. Since our mechanical system could turn 180 degrees in a quarter of a second, the camera ended up being the slowest component by far. 
This was a combination of the camera's hardware limitations and an unknown problem in our software that made image processing much slower than some other teams were able to achieve.

We also should have tested our turret on the other side of the tournament table. In the finals, we ended up on the other side of the table and our thermal camera was thrown off by the bright windows in its field of view.
This was likely a contributing factor to our defeat in deathmatch. If we had more time, we could've programmed a more robust setpoint algorithm to locate a human target rather than finding the centroid of all heat in the frame.

If we accomplished all these improvements (resulting in a super-accurate and super-fast turret/tracking system), we could've also attempted to shoot the target within the 5 second time period in which they were not stationary. 
The speed and accuracy of our Nerf dart would then be the only limiting factor on how likely the opponent is to be able to dodge the shot. We could always upgrade the Nerf gun to a better model if desired.

Overall, however, we are quite pleased with our performance and happy that we learned a lot about how to better design and optimize mechatronic systems.
