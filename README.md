# automated-drop-tower
Here is the control code I developed during my internship in Exactech's engineering lab. I didn't have much coding experience prior to this, 
but the project motivated me to learn as much as I could to get it working as well. I took an interest in signal 
processing and low-level computing during this project, and I successfully applied principles I learned in my control theory classes to 
create a code that read from several sensors, controlled the movement of an electric motor, recorded the number of cycles completed, and 
detected several possible errors. The motor was upgraded from a DC motor and PWM H-bridge setup to an Applied Motion integrated StepSERVO, 
but controlling this motor and running the tft screen HMI proved to be too much for the Arduino platform and a PLC was used instead. I 
understand the frustration of troubleshooting unannotated code written by somebody else, so I make a point to annotate my codes clearly so 
that anyone who looks at it later can understand what different parts do. The @@@ symbols used throughout are meant to indicate parts of 
the code that require troubleshooting. Some of these were not fixed when the Arduino platform was abandoned for this project.

I am very proud of this control code, and I think it serves as a good example of my willingness and ability to learn new and challenging
skills, attention to detail, capacity for troubleshooting, and creativity when solving technical problems. Besides developing the control
code my favorite challenge was designing an infrared photogate. It was one of my first experiences with component-level electronics
and 3-D printing, and it was incredibly satisfying to put the sensor together and successfully read from it. It was used to measure the
speed of the falling impactor so that the test could be aborted in the event of a large variation from the norm. When I first implemented it, I found
that the interrupt routine in the microcontroller was triggered several times whenever the IR beam was broken. Using an oscilloscope, I 
determined that high frequency interference was causing the signal to pass the threshold voltage multiple times at the "edge" of the signal
change. I then used a 555 timer we had on hand in the lab to create a Schmitt trigger and tuned the hysteresis with voltage dividers. It
was incredibly satisfying when I successfully measured the speed of the sled and automatically stopped the test when an error was detected.
This feature made the impact testing machine safer and more reliable, and less likely to destroy valuable test samples.
