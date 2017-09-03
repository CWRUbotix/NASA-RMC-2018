The locomotion.py program is responsible for locomotion part of glennifer
#It performs three simple tasks:
    1. Receive commands from decision layer
    2. Interpret these commands and see if there are any errors
    3. Send commands to individual wheels

#1: Receive commands from decision layer: 
It receives 3 types of commands from decision layer:
#
    1. Straight ---> Going straight
    2. Turn     ---> Turn left or right
    3. Strafe   ---> Strafe
    
#2: Interpret these commands and see if there are any errors
The program do error decetion by looking back the configuration 
for example: the command says go straight, but the wheels are in turn configuration
If there are any errors:
the program will decide whether it can handle it, or send a error notification

#3: Send commands to individual wheels:
The program sends messages to four wheels: front_left, front_right, back_left, back_right
These messages contain wheel rpm and message timeout

By doing this, the program have four different queue for four different purposes:
#
    1. Straight queue ---> only receive straight commands
    2. Turn queue     ---> only receive turn commands 
    3. Strafe queue   ---> only receive strafe commands
    4. Configuration queue ---> only receive information about wheels configuration
    

#4Room for thought:
#
    1. Should the configuration queue be requested by remote procedure queue
    2. What if three queues: straight, turn, strafe be innundated with commands, will there any errors
    3. What kind of variables we will use for pod to turn the wheels, eg: degree??
    4. How we gonna strafe: how the wheels turn to strafe
