# Creation of a connection test plan

## Introduction
This is a test plan designed for testing the connection with the robotic arm.

## To be tested
- Making a connection over UTP with the robotic arm

## Not going to be tested
- Movement of the robot arm
- Anything other than making a connection

# Testing
## Hardware
### Requirements hardware
- Robotic arm
- UTP Cable
- A laptop or desktop (whatever you prefer to carry around to the robot arm)

Before starting the test you will have to install a version of ubuntu 20.**.** LTS [link](https://releases.ubuntu.com/20.04/). You can try it on a VM but preferably it will be a installation of the IOS on your PC (THE VM WILL FOR SURE MAKE TROUBLES WITH THE CONNECTION). Next off you will have to change the bootorder of your device, this is device specific so you would have to lookup your own device manual to find out but most oftenly you can open the bootmenu with F1,F2 or F3. Once this is changed you will boot from the USB and the installation will start. A in depth installation guide regarding the installation can be found [here](https://itsfoss.com/install-ubuntu/)

### Prerequisits before stepping into the software
Plugin the UTP cable from the robotic arm controller into the laptop

## Software
### Prerequisits on the laptop
Considering the steps in hardware have been completed.
- Have this repository installed on your computer
- Share your wifi connection with the robot controller over UTP
  - Accomplish a wifi connection with a wifi point of your liking (eduroam?)
  - Turn on your wired connection
  - Go to IPV4 settings
  - Share with local computers on network 10.42.0.1 subnet 255.255.255.0

### Prerequisits on the robot controller
Setting a static IP
- Turn on the controller
- Move to settings (screenshot here)
- Security > network
- Set static ip to 10.42.0.2 and subnet to 255.255.255.0 (if not already)

### Testing
Move to the appropriate directory
```
./start_connection
```
Execute the command on the computer

As soon as the script is finished run the program on the controller using the > key for remote connection and the connection should establish on the computer
