# AUTOMATED-SELF-PARKING-ROBOT
Automated self parking robot that was coded in C and made using an MSP 432 microcontroller. Uses Infrared Sensing, Interrupts, Ultrasonic Sensors, Servo Motors, Bluetooth and LCD Screens. 

Project Description:  
For my project, I developed a self parking bumper car. This was done by using an MSP432 
microcontroller and the system includes a servo, an ultrasonic sensor, an LCD screen, two DC 
motors, and interrupt-based functionality.  The servo is used to have the ultrasonic sensor be able 
to sweep 90 degrees to the right and -90 degrees to the left . This is needed as the 
ultrasonic sensor needs to be able to sweep left and right to detect which wall is a parking spot, 
and which one is a wall that it should avoid. The ultrasonic sensor is used to detect the initial 
wall, which is a dead-end, then sweep left and right to check which area is a parking space. 
The LCD screen is used to tell the user which state the robot is in, for example if 
it is in the sweeping state it will output the words “Starting Sweep!” The two DC motors are used 
to spin the wheels of the robot for it to move forwards, backwards, left and right. IR sensing was 
used to see if the robot was detecting a black line or not in order for it to follow a dark line . 
After combining all the components, the robot first starts at the starting line, then 
follows a dark line until it detects a wall that is less than 10 cm away. After it detects the wall, 
the robot sweeps right, then left to detect which wall is the closer wall and turns opposite to that 
wall, after it turns into the direction of the parking space it parks itself in the designated location. 
Original Conceptualization: 
My initial vision for this project was for it to be able to follow any line that acts as a road in real 
life, then detects a wall and finds a parking spot. After finding a parking spot, it would use the 
bumper sensors to perfectly park in the parking spot. I was aiming for near perfect precision that 
imitates modern self parking cars such as Teslas. I wanted it to be simple, yet effective. 
Obstacles Faced: 
Throughout the development process, I encountered many challenges. The biggest challenge in 
my books was the actual detection of a parking spot and near perfect parking. I first tried to have 
the bumper car be able to line up in the parking spot perfectly, but due to there only being 
bumper sensors on the front of the robot, I was very limited in the precision of parking. This 
made me have to make the project way too simple and because of this problem, I was not as 
satisfied with this project. 
Testing and Verification: 
I have conducted near hundreds of trial runs to see if the car would park in the designated 
parking spot correctly, and if it would turn to the correct direction since the walls were not very 
far apart, I though the IR sensor might have a tough time sensing the correct distance as the servo 
was not perfect as well leading to the IR sensor sometimes not being perfectly perpendicular to 
the walls it had to detect, but alas this was not a problem since the hypotenuse of the lines were 
decently far enough. 
Possible Improvements: 
For future iterations, several enhancements could be made. I could have made this exact same 
project, but added bumper sensors to the back of the robot as well, but this would not have been 
possible at the time of doing this since there were no available modules for that. I could have 
also used a possibly better servo that is more accurate to ensure that the IR sensor would be 
perfectly perpendicular to the wall. Additionally, integrating a bluetooth system where I could 
control the robot as if it were to be an RC car, then having myself control what state the robot is 
in, just as if you were in a real car, but sometimes having the robot intervene if the user makes a 
mistake.
