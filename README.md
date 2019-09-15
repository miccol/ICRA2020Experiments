



# Simulated Experiments for the paper: Act, Perceive, and Plan in Belief Space for Robot Localization.

In this repository, you will find the code related to the experiments of the paper  "Act, Perceive, and Plan in Belief Space for Robot Localization" Submitted at ICRA2020  by Michele Colledanchise, Damiano Malafronte, and Lorenzo Natale.

## Dependencies

[Eigen](http://eigen.tuxfamily.org/)

[Python3](https://www.python.org/downloads/) with [pygame](https://www.pygame.org) (optional, for on-screen visualization)



## Setup

Open a terminal and clone the repository

```bash
$ cd /path/to/folder`
$ git clone https://github.com/miccol/ICRA2020Experiments.git
```

Once you have the repository, compile the library:

### Gnu/Linux or MacOS
In Unix-based operating system, open a terminal and run the following commands:
```bash
`$ cd /path/to/folder/`
`$ mkdir ./build`
`$ cd build`
`$ cmake ..`
`$ make`
```

### Windows
In Windows, run the`[https://cmake.org](https://cmake.org/)`.
Open the visual studio solution file (`.sln`)



## Execution instructions:

The process above installs two executables, one to run a single experiment and one compute the performance comparison described in the paper. 

#### Run Single Experiment

The executable `run_single_experiment` runs our approach on user-defined environment.

To set up the environment open the file `run_single_experiment.cpp` and edit the following code:



```c++
const int size_x = 5;
const int size_y = 5;
    
const int initial_physical_state = 0;    
    
const std::vector<int> states_with_A = {0,1,2,3};
const std::vector<int> states_with_B = {4,5,6,7};
const std::vector<int> states_with_C = {8,9,10,11};
const std::vector<int> states_with_D = {12,13,14,15};
```

Set the grid size (`size_x`and `size_y`), the initial (unknown to the robot) physical state (`initial_physical_state`) and the physical states with the object classes `A`,  `B`,  `C`,  `D` that can be seen by the action `Look `.   

The physical states have the following numbering , the bottom left triangle facing right is 0, the bottom left triangle facing up is 1, the bottom left triangle facing left is 2, the bottom left triangle facing down is 3 and so on moving to the next cell to the right and then to the next row, as in the figure below. 



![indexed](https://user-images.githubusercontent.com/8132627/64920553-55ce0f00-d7b9-11e9-9fa3-7cde582f7575.png)





Once you have set up the environment, build and run the executable. If you have python with Pygame installed, you will also see the actions executed on screen, as in the following animated gif.



![executonexample](https://user-images.githubusercontent.com/8132627/64911918-28855080-d728-11e9-9bd1-fe29219a112b.gif)

The blue dot is the physical robot state. Shades of gray represent the probability of being on that state. 

The screenshots of the execution steps are also saved in the build directory for inspection.



####  Run performance comparison

The executable `run_comparison` runs the comparison described in the paper and shows results on the terminal. The exact result may differ depending on the computer setup, however the considerations done in the paper should match.



## LICENSE

The MIT License (MIT)

Copyright (c) 2019 Michele Colledanchise

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

