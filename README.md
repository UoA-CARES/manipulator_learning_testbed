# Manipulator Learning Testbed

## General Configuration

The following image illustrates the manipulator configuration, where you can clearly see the ID and name assigned to each motor. Please note the voltage and bps for this configuration too. Please ***do not change the ID settings and the communication speed*** so we can all work under the same parameters

![](https://github.com/UoA-CARES/manipulator_learning_testbed/blob/main/Images/configuration.jpg)


## State Space generation and basic math

This scheme describes the variables that should make up the state space following Henry's previous explanation. As you can see, the diagram has a reference frame from which all measurements start. In addition, you will have the frame of the cube and the frame of the target point. The position of each joint and the extreme effector are described in the equations (w.r.t. the reference frame too). 

![](https://github.com/UoA-CARES/manipulator_learning_testbed/blob/main/Images/general%20math-14.jpg)

the code to generate the space state following this schema can be found [here] (https://github.com/UoA-CARES/manipulator_learning_testbed/blob/main/Code/state_space_generator.py)
