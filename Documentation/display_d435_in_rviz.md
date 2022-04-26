# Testplan period 4

## Introduction
This is a testplan written for the robotic arm

## To be tested
- octomap in from the real camera in rviz

## Testing

### Installation of steps
For the install steps follow the steps the command on github [link](https://github.com/SmartFarmingPerenMinor/SFP-TreeCutter/blob/master/install.sh) or run **install.sh**.

### Starting the simulation

Move to the correct directory
```bash
cd ~/SFP-TreeCutter
```

Run the start script 
```bash
./startrvizgazebo.sh
```

#### Importing display the ur10e in Rviz
Under **Display**
        select **Fixed Frame** -> **base_link**

__click__ on **add** -> **MotionPlanning**
        press the button with the **check mark**

## Criteria for succes
#### Download criteria
- Can you download the files?

#### setup the file criteria
- Does the setup script let the tree appear? yes or no?

#### Start criteria
- Did the ./startgazebo script automatically start the simulator?
- Can you see the arm in the world?
- Can you see the tree in the world?
  - If not was the setup script run in SFP-TreeCutter/catkin_ws/src/test/worlds
