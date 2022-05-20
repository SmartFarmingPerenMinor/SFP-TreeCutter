# Test scenario's

## Introduction
This is a document meant to display multiple scenario's that could happen in real life or are just for practical use.

## Scenario's
- Someone gets strangled in the robotic arm and the red button is pressed
- Someone has to get close to the robotic arm and pauses the execution
- Does the robot move straight to the point or do the joints have any influence on the way it moves

## Scenario 1

The unfortunate event happens the unthinkable and the robot arm needs to be stopped immediatly. What happens when you press the red button?

### Things that are to be tested
- How long does it take for the robotic arm to come to a stand still?
- What is the delay between pressing the button and something happening?
- How long does it take to restart the entire program?

### How is it going to be tested?
- 

## Scenario 2

Something gets stuck or someone needs to get in the proximity of the working range of the arm, this means for safety the robotic arm should be paused in its execution. Luckily the robotic controller has a pause button that can execute this task

### Things that are to be tested
- How long does it take before you can enter the immediate proximity of the robotic arm?
- How long does it take to restart the entire solution to continue the program?

### How is it going to be tested?
A program is to be written that can detect whenever code execution is paused

## Scenario 3

How does the robot actually move? Are we safe to say that the robot moves directly to the points we assign or does it move along its joints to improve speed in execution?

### Things that are to be tested
- Robot movement according to joint max speeds

### How is it going to be tested?
The robot will get a laser attached to its end effector. The laser will be pointed at the ground and the robot will move along the horizontal lines and vertical lines. For the horizontal lines if the robot doesn't stay within the taped line we can tell that it is moving according to the joints optimal speeds. For the vertical line we can see if the dot moves over away from its static position on the ground.