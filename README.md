# automated-drop-tower
Here is the control code I developed during my internship in Exactech's engineering lab. I taught myself the Arduino language in a few 
months; my only programming experience before this was creating simple MATLAB and LabVIEW programs. I took a strong interest in signal 
processing and low-level computing during this project, and I successfully applied principles I learned in my control theory classes to 
create a code that read from several sensors, controlled the movement of an electric motor, recorded the number of cycles completed, and 
detected several possible errors. The motor was upgraded from a DC motor and PWM H-bridge setup to an Applied Motion integrated StepSERVO, 
but controlling this motor and running the tft screen HMI proved to be too much for the Arduino platform and a PLC was used instead. I 
understand the frustration of troubleshooting unannotated code written by somebody else, so I make a point to annotate my codes clearly so 
that anyone who looks at it later can understand what different parts do. The @@@ symbols used throughout are meant to indicate parts of 
the code that require troubleshooting. Some of these were not fixed when the Arduino platform was abandoned for this project.

I am very proud of this control code, and I think it serves as a good example of my willingness and ability to learn new and challenging
skills, attention to detail, capacity for troubleshooting, and creativity when solving technical problems.
