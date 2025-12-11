# <u>GALAGA GAME IN ROS1</u>


Marcos Granados, Carlos Lázaro and Hugo Cárdenas  

## ROS Noetic Installation
Install ROS Noetic in the device using the official ROS wiki.

## Catkin Initialization
Catkin enviroment should be initialized and built for the game to work. The package files should be with the other packages inside the directory catkin_ws/src/


## ROS Nodes
The game is run using ROS node:

- 'GAME_NODE' : Main node that runs the game. When activated, it opens a window that displays the game.
- 'INFO_USER' : Node used to begin to play. It requires information about the user and sends it to other nodes that require it.
- 'CONTROL_NODE' : Node that handles the instructions from the user. Comunicates with GAME_NODE.
- 'RESULT_NODE' : Once game is finished, it requests the username and displays the obtained score if correct.
- 'DIFFICULTY_NODE' : Node used to change te difficulty level between easy, medium and hard. Comunicates with GAME_NODE


## ROS Topics
The topics used in the implementation are:

- '/user_information' : Information about the user. Nodes GAME_NODE and RESULT_NODE subscribe to it.
- '/keyboard_control' : Instructions to control the game. Node GAME_NODE subscribes to it.
- '/result_information' : Scores. Node RESULT_NODE subscribes to it.
- '/color_information' : Integer with the code for a specific color. GAME_NODE node subscribe to it.


## ROS Services
The services used in the implementation are:

- 'score' : Sends the score information only when a play has finished. Client in RESULT_NODE request a service in GAME_NODE.
- 'difficulty' : Sends the difficulty change petition. Only possible in the welcome screen. Client in DIFFICULTY_NODE requests a service in GAME_NODE.

## ROS Parameters
The parameters used in the implementation are:

- 'user_name' : Stores user name.
- 'change_player_color' : Integer value with the color.
- 'screen_param' : Screen state that it executed.

## Requirements to Play
A roscore comand must be called in a terminal. In aditional terminals, the different nodes should be called using rosrun method  
An aditional way to play is calling the launch file with roslaunch. To achieve this gnome-terminal should be installed in the enviroment (to allow openning several terminals from one single launch file.)


## How to Play?
When starting a game, the window will not allow any movements until the player has entered their information in the INFO_USER node.    
Later, all the fucntions can be controlled with the CONTROLLER_NODE Node. There are instructions along the game, so the player can understand each of the key instructions, but the basic are:

- 'Arrow keys' : to move left or right.
- 'Space bar' : to shoot.
- ''P' key' : to pause mid-game.  
Aditional instructions will come with a explanation in the game window.  

If the player has finished a game, they can see their score in the RESULT_NODE node by typing the username they used before.  
If the player is in the welcome screen, they can change the difficulty level using the DIFFICULTY_NODE node.
There is also the possibility to change the color using the COLOR_NODE. Options are 1: Purple, 2: Green, 3: Orange.


