
  
  
  

## String Plotter

  

![image](https://github.com/DrGlaucous/HeavyPlotter/raw/main/Images/Flintstones.jpg)

  

---

  This is the repo for a whiteboard mounted string plotter, or *polarograph*.
It is designed to use standard 3D printer hardware and suction cups to mount to virtually any smooth surface or whiteboard. 
When not in use, it compacts nicely into a shoebox for easy transportation.

---
The plotter itself is not particularly fancy or innovative. Better designs probably exist.

What makes this plotter unique is that it makes use of the ESP32 and [FluidNC](https://github.com/bdring/FluidNC) for control. 

This allows versatile control over how the plotter will behave and a whole host of CAM software for generating plots, all wirelessly.

Several modifications have been made to the FluidNC firmware to optimize its use for this application.
New Commands:
* **$SAD = [distance in mm]** *Set Anchor Distance: set the distance between left and right anchor points when the plotter is set up.*
* **$DMC = [1/0]** *Direct Motor Control: Jogging with X and Y will move each motor directly without any kinematic conversion, to be used in conjunction with **\$CGL** *
* **$CGL = [0/1]** *Calibrate Gondola Location: 0 for left, 1 for right. Indented to help the plotter calculate the distance between anchors, but I find that using a tape measure and the **\$SAD** command is quicker and easier.*

The commands assume the plotter's 0,0 is directly below the left anchor. How far down can be set in the config file (the one I used can be found in the firmware folder with the fluidNC build itself).

---
Hardware:

* For the controller, I used a ESP32 Lolin32 Lite board. These can be picked up for extremely cheap online compared to other ESP32 models.
* I used two A4988 stepStick modules to control the steppers, which were just what I had on hand. Any type of motor will work, so long as it has somewhat decent torque and a NEMA17 form factor.
* A cheap 9g servo is used to actuate the marker.
* A buck-boost regulator supplies 3.3 volts to the controller and servo.
* I used a refitted 12V laptop power supply to give the contraption power, but it should be able to work on any voltage within the stepper controller and regulator's range. (higher voltages will mean more holding torque from the motors.)
