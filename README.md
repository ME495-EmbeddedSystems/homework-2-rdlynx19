# ME495 Embedded Systems Homework 2
Author: Pushkar Dave

Visualises an arena with a holonomic turtlebot, which catches a brick and drops it at the center of the arena.

## Quickstart
1. Use `ros2 launch turtle_brick turtle_arena.launch.xml` to start the arena and turtle simulation
2. Use `ros2 service call /drop turtle_brick_interfaces/srv/Drop` to drop a brick
3. Here is a video of the turtle when the brick is within catching range
   
https://github.com/user-attachments/assets/09a923c5-6b5e-4a33-8129-4b686d6486cf

5. Here is a video of the turtle when the brick cannot be caught

   ${embed video here, it must be playable on github. Upload the video as an issue and link to it}
