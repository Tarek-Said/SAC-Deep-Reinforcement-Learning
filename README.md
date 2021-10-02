# Soft Actor Critic Reinforce Learning for autonomous manipulator using Gazebo and MATLAB

In this tutorial, we will explain how to built successfully a model of SAC-DRL to train Kinova robot arm to manipulate in an enviroment.

Before we start, please download and install [***VMware player***](https://www.vmware.com/go/getplayer-win) virtual machine player. And [***ros_melodic_dashing_gazebov9_linux_win_v3.zip***](https://ssd.mathworks.com/supportfiles/ros/virtual_machines/v2/ros_melodic_dashing_gazebov9_linux_win_v3.zip) virtual machine.

we would suggest you to practice the following example:

- [***Perform Co-Simulation between Simulink and Gazebo***](https://www.mathworks.com/help/robotics/ug/perform-co-simulation-between-simulink-and-gazebo.html?searchHighlight=gazebo%20co%20simulation%20&s_tid=srchtitle)

This example will help you to understand the way to obtain data from gazebo and send comands as well.


## Environment setup

1) Open the ***VMware player*** and add the ***virtual machine*** to it by clicking on `Player/File/Open`.
2) Select the virtual machine and click on `Play` button.
    - Check that the following code is added to the `.world` file that you are working with. or,
    - If you are working with different gazebo invironment, add the following code to `.world` file in your workspace or package.

```xml
<plugin name="GazeboPlugin" filename="lib/libGazeboCoSimPlugin.so"><portNumber>14581</portNumber></plugin>
```
![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/plugin.png)

3) From `Desktop`, run `Example World 1` file and wait for gazebo to launch.

## Simulink block diagrame

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Gazebo%20Pacer.png)

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Connect%20to%20gazebo.png)

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Gazebo%20Read%20Object%20Position.png)

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Observations.png)

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Joint%20actions.png)

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Reward%20function.png)

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Termination%20function.png)
