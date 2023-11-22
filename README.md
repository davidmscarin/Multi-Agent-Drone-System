## Multi Agent Autonomous Drone Delivery System

The script "multi_agent_drone_system.py" simulates a network of Drone and Hub agents using SPADE and an XMPP serve for communication. The Hubs store packages and communicate with Drone Agents to pick them up and deliver them to their location. The Drone agents also communicate between themselves to coordinate deliveries, avoid crashes and help with random problems. 

All of the information about the deliveries, positions and agent attributes are stored in the Environment which the Agents can interact with, perceive and change.

While running the script various lines are printed to the terminal detailing what is happening in the simulation, including positions of the agents, their current tasks and statuses and communications between them. At the end of the runtime some metrics used to evaluate the drone's performance will be displayed as a bar graph as well as a 3D animation of the drone's movements during this iteration of the code using matplotlib. Furthermore, there will be displayed a graphical interface with descriptive icons using pygame.

Files needed:
- multi_agent_drone_system.py: main script
- Interface.py: script to display the interface
- wireframe.py: script to implement the wireframe for the interface
- images: image folder
