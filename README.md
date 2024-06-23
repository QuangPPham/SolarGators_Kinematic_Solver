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

Coordinates are defined from the center of the front wheels. +x is rearward, +y is outward (from right wheel), and +z is upward. See the Solidworks file in the sketches folder for where the coordinates system is located. Note that the wheel shown in this file is the left wheel, so the y coordinate input into the function will have the opposite sign.
