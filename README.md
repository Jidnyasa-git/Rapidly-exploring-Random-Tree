# Rapidly-exploring Random Tree
## What is RRT ?
Rapidly-exploring random tree (RRT) is a sampling based path planning algorithm. RRT is designed to efficiently search nonconvex, high-dimensional spaces by randomly building a space-filling tree. The tree is constructed incrementally from samples drawn randomly from the search space and is inherently biased to grow towards large unsearched areas of the space. It easily handles problems with obstacles and differential constraints (nonholonomic and kinodynamic) and have been widely used in autonomous robotic motion planning.
## Algorithm
```
1. Initialize search tree T with x(start)
2. while T is less than the maximum iteration do
3.      x(sample) <-- sample from X (i.e. point from available space)
4.      if x(sample) is not in the obstacle
5.           continue
6.      x(nearest) <-- nearest node in T from x(sample)
7.      find x(new) which is at a distance of stepsize from x(nearest) in the direction of x(sample)
8.      if the edge joining x(nearest) and x(new) does not intersesct with the obstacle
9.           continue
10.     add x(new) to T with an edge from x(nearest) to x(new)
11.     if x(new) is in x(goal)
12.          return path from start to gaol
13. return path from start to goal
```
## Requirements
C++ 11+ <br />
SFML (Simple and Fast Multimedia Library)
## Install SFML
Run the following command to install SFML on linux
```
$ sudo apt-get install libsfml-dev
```
## Compile and Run
```
$ g++ -std=c++11 -c rrt.cpp
$ g++ rrt.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system
$ ./sfml-app
```
## SFML Output

 Output window size <br />
 Width : 1000 pixels <br />
 Height : 1000 pixels <br />
 <br />
 Start point : Light green node <br />
 Goal point : Purple node <br />
 Other nodes : Blue <br />
 Path from start to end point : Red edges <br />
 Other edges : white <br />
## Example 
```
Goal point as x y : 250 750
```
```
Start point as x y : 350 350
```
```
Enter number of obstacles : 6
```
```
Enter number of vertices in obstacle 1 (more than 2): 4
Enter x y coordinates of vertices in clockwise or anti-clockwise manner 
Vertex 1 : 200 150 
Vertex 2 : 200 200
Vertex 3 : 800 200
Vertex 4 : 800 150
```
```
Enter number of vertices in obstacle 2 (more than 2): 4
Enter x y coordinates of vertices in clockwise or anti-clockwise manner 
Vertex 1 : 200 200
Vertex 2 : 200 400
Vertex 3 : 300 400
Vertex 4 : 300 200
```
```
Enter number of vertices in obstacle 3 (more than 2): 4
Enter x y coordinates of vertices in clockwise or anti-clockwise manner 
Vertex 1 : 200 400
Vertex 2 : 200 450
Vertex 3 : 800 450
Vertex 4 : 800 400
```
```
Enter number of vertices in obstacle 4 (more than 2): 4
Enter x y coordinates of vertices in clockwise or anti-clockwise manner 
Vertex 1 : 0 650
Vertex 2 : 0 700
Vertex 3 : 400 700
Vertex 4 : 400 650
```
```
Enter number of vertices in obstacle 5 (more than 2): 4
Enter x y coordinates of vertices in clockwise or anti-clockwise manner 
Vertex 1 : 300 700
Vertex 2 : 300 850
Vertex 3 : 400 850
Vertex 4 : 400 700
```
```
Enter number of vertices in obstacle 6 (more than 2): 4
Enter x y coordinates of vertices in clockwise or anti-clockwise manner 
Vertex 1 : 650 700
Vertex 2 : 650 1000
Vertex 3 : 750 1000
Vertex 4 : 750 700

```
<img src="https://github.com/Jidnyasa-git/Rapidly-exploring_Random_Tree/blob/main/rrt.PNG" width="750">
