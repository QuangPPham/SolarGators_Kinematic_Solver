# SolarGators_Kinematic_Solver
Kinematics Solver for Suspension geometry  
By: Quang Pham, 2024

Run by writing in the command line in the folder where the scripts are located:  
```  
python main.py
``` 

Input: coordinates of the below points for the right wheel. See main.py for example usage.   
   * FG: initial wheel center (x,y,z)   
   * SP_in: spring chassis attachment  
   * SP_out: spring rocker attachment  
   * R1: rocker axis 1 (point on the axis of rotation of the rocker)  
   * R2: rocker axis 2 (2nd point on the axis of rotation of the rocker)  
   * PR_in: push rod rocker attachment  
   * PR_out: push rod knuckle attachment  
   * TR_in: tie rod inboard attachment  
   * TR_out: tie rod upright attachment  
   * SA: spindle axis (point on the axis of rotation of the wheel)  
   * UB: upper ball joint  
   * LB: lower ball joint  
   * WB: contact patch  
   * UCF: upper front inboard  
   * UCR: upper rear inboard  
   * LCF: lower front inboard  
   * LCR: lower rear inboard  

Coordinates are defined from the center of the front wheels. +x is rearward, +y is outward (from right wheel), and +z is upward. See the Solidworks file in the sketches folder for where the coordinates system is located. Note that the wheel shown in this file is the left wheel, so the y coordinate inputs into the main.py script will have the opposite sign.  

The suspension movement is simulated by changing the length of the spring, which then moves the rest of the suspension points correspondingly. The new suspension points are calculated via a process called trilateration, where you can figure out the coordinates of a 4th point by knowing the coordinates of 3 points and the length of each of them to the 4th point. The suspension parameters (camber, castor, etc.) are then calculated from these suspension points. The book "The Multibody Systems Approach to Vehicle Dynamics" by Mike Blundell and Damien Harty is a good resource, and I included the pdf file here.  Chapter 2 shows how to simulation suspension motion, and Chapter 4 shows how to calculate the parameters.
