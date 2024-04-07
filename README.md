# Dijkstra Pathfinding for Point Robot

## Description
This project implements Dijkstra's algorithm to find the shortest path for a point robot navigating a 2D space with obstacles. The robot is allowed to move in 8 directions, and obstacles are defined within the space to test the algorithm's effectiveness. The final path and the exploration process are visualized and saved as a video file.

## Installation

Before running the program, ensure you have Python installed on your system. This program was tested with Python 3.8. Additionally, you'll need to install the following libraries:

- NumPy
- OpenCV
- heapq (part of Python's standard library, no need for installation)

You can install NumPy and OpenCV using pip:

```
pip install numpy opencv-python
```
Enter the starting coordinates in the "Start node" in the terminal
Enter the Goal node in the "End node" in the terminal
Example:
Start node:10 10 
End node:1190 490
## Running the Program

To run the program, navigate to the directory containing the script and execute the following command in your terminal or command prompt:

```
python dijkstra_rohit_suresh.py
```

## User Inputs

Upon running the script, you will be prompted to enter the start and end nodes for the pathfinding algorithm. The coordinates should be entered in the format `x y`, where `x` is the horizontal position and `y` is the vertical position. Please note the following:

- Ensure the coordinates are within the bounds of the map, which is 1200 pixels in width and 500 pixels in height.
- The coordinate system's origin `(0,0)` is located at the bottom-left corner of the map.
- Consider the clearance of 5mm around obstacles when choosing start and end points.
- Invalid entries, such as those within obstacles or out of bounds, will prompt you to re-enter the coordinates.

## Output

After processing, the program generates a visualization of the explored nodes and the final path, which is saved as a video file named `djkstra_rohit_suresh.mp4` in the same directory as the script. The console will display the total cost of the path and the execution time of the program.

## Libraries/Dependencies

- NumPy: Used for efficient array operations.
- OpenCV (cv2): Utilized for creating and manipulating the visual representation of the map and pathfinding process.
- heapq: A Python built-in library that implements a priority queue, used for selecting the next node to explore based on the shortest distance.

---
