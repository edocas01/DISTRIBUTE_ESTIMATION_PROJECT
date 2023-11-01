# DISTRIBUTE_ESTIMATION_PROJECT
This repository contains the code developed by Edoardo Castioni and Massimiliano Pranzo for the project of the "Intelligent distributed system" of professor Fontanelli.
The project consist in the distributed encirclement of a moving target (person, wheeled robot, ...) via a set of agents that can interact with each other in a confined range. The main problem is to dispose the agents in a predefined formation and equally spaced. The whole algorithm is developed in a distributed manner. <br>
## Video of the simulation
<p align="center">
<img src="IMAGES/INITIAL_VIDEO/video.gif" width="700" height="350"/>
</p>

## Code Management
The code is writte in [_Matlab_](https://www.mathworks.com/products/matlab.html) and it is organized in the following way:
- [_Classes_](/Classes/) contains the definition of the classes used in the project:
  - `ROBOT.m` is the class that defines the agents.
  - `TARGET.m` is the class that defines the target.
  - `OBSTACLE` is the class that defines the moving obstacles. 
  - `LARGE_OBSTACLE` is the class that defines the static obstacles (walls).
- [_Functions_](/Functions/) contains the functions used in the project.
- `config.m` is the file that contains the parameters of the simulation, this file has to be tuned according to the user needs.
- `main.m` is the main file of the repository. To start a simulation it is sufficient to run this file.
- [_Scripts_](/Scripts/) and [_Scripts_for_report_](/Scripts_for_report/) contain the scripts used to generate the results of the report and to test the functions (mainly for plots and debugging purposes).
- [_Results_](/Results/) and [_IMAGES_](/IMAGES/) contain the results of the simulations with images and videos.

# Papers and related works
These section contains some of the papers and the related works that have been used to develop the project.
## General papers
- [_Distributed Optimal Deployment on a Circle for
Cooperative Encirclement of Autonomous
Mobile Multi-Agents_](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9044350)
- [_Distributed encirclement control of multi-agent
systems_](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7231770)
- [_Distributed Kalman Filters for Relative
Formation Control of Multi-agent systems_](https://arxiv.org/pdf/2110.06332.pdf)
- [_Proportional-Integral Formation Control for Multi-Agent Systems with
Time-Varying Attitudes under Relative Measurements_](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8899676)

## Voroni tassellation
- [_Multi-Robot Formation Control Based on CVT Algorithm and Health Optimization Management_](https://www.mdpi.com/2076-3417/12/2/755)

## Distributed Kalmann Filter
- [_Distributed Kalman Filters for agents consensus_](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7434174)


