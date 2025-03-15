Extensively commented in the code. 
Provide input x and y coordinates for start node first, then provide for goal node
If either of the start and/or goal nodes are in the obstacle and/or clearance space, a message will be returned stating that the start/goal nodes are not located in the accessible space.
For this code, I used OpenCV to visualize the bfs pathfinding algorithm and display output
Used numpy array to create obstacle and clearance masks which could then be used to assign pixel colors (black for obstacle, blue for clearance and white for accessible pixels)
Used queue module to store parent and neighbor nodes in a manner that can allow for proper implementation of BFS code
