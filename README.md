PlanOpt: Translation interface approach for Task and Motion Planning
====================================================================

Solves task and motion planning problems by using an interface which translates task plans to motion planning problems

Dependencies
============
python2.7, gurobipy, numpy, eddie-col branch of trajopt

Setup Instructions
==================
Install the eddie-col branch of trajopt
Trajopt installation instructions: http://rll.berkeley.edu/trajopt/doc/sphinx_build/html/install.html

Clone PlanOpt
```
$ git clone https://github.com/c-l/planopt.git
```

To set up the task planner follow the instructions in the readme in the planners/myFDFiles folder.

Example
=======

Generate World
```
$ cd src
$ python ../envs/world.py
```

Run Planner
```
$ python hybridPlanner.py -v -d tc -e ../envs/twocan_world.dae
```
