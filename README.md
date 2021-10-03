# Soft Actor Critic Reinforce Learning for autonomous manipulator using Gazebo and MATLAB

In this tutorial, we will explain how to built successfully a model of SAC-DRL to train Kinova robot arm to manipulate in a 3D enviroment. The objective of the SAC is learn how to control joints velocity to reach the target position.

Before we start, please download and install [***VMware player***](https://www.vmware.com/go/getplayer-win) virtual machine player. And [***ros_melodic_dashing_gazebov9_linux_win_v3.zip***](https://ssd.mathworks.com/supportfiles/ros/virtual_machines/v2/ros_melodic_dashing_gazebov9_linux_win_v3.zip) virtual machine.

we would suggest you to practice the following example:

- [***Perform Co-Simulation between Simulink and Gazebo***](https://www.mathworks.com/help/robotics/ug/perform-co-simulation-between-simulink-and-gazebo.html?searchHighlight=gazebo%20co%20simulation%20&s_tid=srchtitle)

This example will help you to understand the way to obtain data from gazebo and send comands as well.


## 1) Environment setup

1) Open the ***VMware player*** and add the ***virtual machine*** to it by clicking on `Player/File/Open`.
2) Select the virtual machine and click on `Play` button.
    - Check that the following code is added to the `.world` file that you are working with. or,
    - If you are working with different gazebo invironment, add the following code to `.world` file in your workspace or package.

```xml
<plugin name="GazeboPlugin" filename="lib/libGazeboCoSimPlugin.so"><portNumber>14581</portNumber></plugin>
```
![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/plugin.png)

3) From `Desktop`, run `Example World 1` file and wait for gazebo to launch.

## 2) Simulink block diagrame

This part shows the the simulink setup in `kinova.slx`.

* Part 1: gazebo Pacer.
* Part 2: it obtains the object position (yellow can) (X, Y, Z).
* Part 3: to get the grapper fingers position.
* Part 4: to calculate the distance reward.
* Part 5: observe joints velocity and angle.
* Part 6: the function that terminates the training episode.
* Part 7: sending the output actions from the SAC to gazebo.

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Kinova.jpg)

Double click on `gazebo pacer` and gonfigure the `ip adress` of the virtual machine. If the plugin works fine, it should connect successfully to gazebo.

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Gazebo%20Pacer.png)

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Connect%20to%20gazebo.png)

As in part 5, the SAC is setted to observe information from joints 1, 2, and 4.

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Observations.png)

While SAC is training, joints velocity are the outputs of the SAC to control the robot. And it ranges from [-1, 1].

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Joint%20actions.png)

The reward function is calculated using the normal distripution function.

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Reward%20function.png)

Termination function is what ends the training episode either the target is acheived or limits are being crossed.

![](https://github.com/Tarek-Said/SAC-Deep-Reinforcement-Learning/blob/main/Termination%20function.png)


## 3) MATLAB script

Configre time for each training episode `Tfinal` with the sample time. Note that the time for each step = `Tfinal` / `sampleTime`.

```MATLAB
mdl = "kinova";
Tfinal = 10;
sampleTime = 0.1;

agentBlk = mdl + "/Agent";
open_system(mdl)
open_system(mdl + "/Environment")
```

Define the number of observations wich are in our case, 3 joints velocity + 3 distance errors for X Y Z + comulative reward.

Also, state the upper and the lower limits of the collected observations.

```MATLAB
% define observation parameters...
observation=10;
obsInfo = rlNumericSpec([observation 1],...
    "LowerLimit",ones(observation,1),...
    "UpperLimit",ones(observation,1));

numObservations = obsInfo.Dimension(1);
```

As what has been done with observations, actions parameters should be defined as well.

```MATLAB
% define action parameters
numActions = 3;
actInfo = rlNumericSpec([numActions 1],...
    "LowerLimit",-1,...
    "UpperLimit",1);
```

Specify the model, agent block name, observation, and action variables to `rlSimulinkEnv`.

```MATLAB
% build environment interface
env = rlSimulinkEnv(mdl,agentBlk,obsInfo,actInfo);
env.UseFastRestart = "off"
```


```MATLAB
% SAC agent-critic network 
% critic 1
statePath1 = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(400,'Name','CriticStateFC1')
    reluLayer('Name','CriticStateRelu1')
    fullyConnectedLayer(300,'Name','CriticStateFC2')
    ];
actionPath1 = [
    featureInputLayer(numActions,'Normalization','none','Name','action')
    fullyConnectedLayer(300,'Name','CriticActionFC1')
    ];
commonPath1 = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu1')
    fullyConnectedLayer(1,'Name','CriticOutput')
    ];

criticNet1 = layerGraph(statePath1);
criticNet1 = addLayers(criticNet1,actionPath1);
criticNet1 = addLayers(criticNet1,commonPath1);
criticNet1 = connectLayers(criticNet1,'CriticStateFC2','add/in1');
criticNet1 = connectLayers(criticNet1,'CriticActionFC1','add/in2');
```

```MATLAB
%critic 2
statePath2 = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(128,'Name','CriticStateFC1')
    reluLayer('Name','CriticStateRelu1')
    fullyConnectedLayer(64,'Name','CriticStateFC2')
    ];
actionPath2 = [
    featureInputLayer(numActions,'Normalization','none','Name','action')
    fullyConnectedLayer(64,'Name','CriticActionFC1')
    ];
commonPath2 = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu1')
    fullyConnectedLayer(1,'Name','CriticOutput')
    ];

criticNet2 = layerGraph(statePath2);
criticNet2 = addLayers(criticNet2,actionPath2);
criticNet2 = addLayers(criticNet2,commonPath2);
criticNet2 = connectLayers(criticNet2,'CriticStateFC2','add/in1');
criticNet2 = connectLayers(criticNet2,'CriticActionFC1','add/in2');
```

```MATLAB
% create  critic using rlRepresentation

criticOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-3,... 
                                        'GradientThreshold',1,'L2RegularizationFactor',2e-4)
critic1 = rlQValueRepresentation(criticNet1,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);
critic2 = rlQValueRepresentation(criticNet2,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);
```

```MATLAB
% create actor
statePath = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(400, 'Name','commonFC1')
    reluLayer('Name','CommonRelu')];
meanPath = [
    fullyConnectedLayer(300,'Name','MeanFC1')
    reluLayer('Name','MeanRelu')
    fullyConnectedLayer(numActions,'Name','Mean')
    ];
stdPath = [
    fullyConnectedLayer(300,'Name','StdFC1')
    reluLayer('Name','StdRelu')
    fullyConnectedLayer(numActions,'Name','StdFC2')
    softplusLayer('Name','StandardDeviation')
    ];

concatPath = [
    concatenationLayer(1,2,'Name','GaussianParameters')
    tanhLayer('Name','tanh')
    scalingLayer('Name','Scaling','Scale',actInfo.UpperLimit)
%     scalingLayer('Name','StandardDeviation')
    ];

actorNetwork = layerGraph(statePath);
actorNetwork = addLayers(actorNetwork,meanPath);
actorNetwork = addLayers(actorNetwork,stdPath);
actorNetwork = addLayers(actorNetwork,concatPath);
actorNetwork = connectLayers(actorNetwork,'CommonRelu','MeanFC1/in');
actorNetwork = connectLayers(actorNetwork,'CommonRelu','StdFC1/in');
actorNetwork = connectLayers(actorNetwork,'Mean','GaussianParameters/in1');
actorNetwork = connectLayers(actorNetwork,'StandardDeviation','GaussianParameters/in2');
```

```MATLAB

% create Stochastic actor representation
actorOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-3,...
                                       'GradientThreshold',1,'L2RegularizationFactor',1e-5);

actor = rlStochasticActorRepresentation(actorNetwork,obsInfo,actInfo,actorOptions,...
    'Observation',{'observation'});
```

```MATLAB
% create SAC agent
SampleTime = 0.1
agentOptions = rlSACAgentOptions;
agentOptions.SampleTime = SampleTime;
agentOptions.EntropyWeightOptions.EntropyWeight = 0.1; 
% agentOptions.EntropyWeightOptions.LearnRate = 0;
agentOptions.PolicyUpdateFrequency = 1;
agentOptions.CriticUpdateFrequency = 1;
agentOptions.DiscountFactor = 0.99;
agentOptions.TargetSmoothFactor = 1e-3;
agentOptions.ExperienceBufferLength = 1e5;
agentOptions.MiniBatchSize = 256; %128;
agentOptions.NumStepsToLookAhead = 3;
agentOptions.SaveExperienceBufferWithAgent =true;
agentOptions.ResetExperienceBufferBeforeTraining = false;
%agentOpts.NoiseOptions.Variance = 0.1;
%agentOpts.NoiseOptions.VarianceDecayRate = 1e-5;
%% Create SAC agent using actor, critics, and options
agentOptions
Agent = rlSACAgent(actor,[critic1 critic2],agentOptions);
open_system(mdl + "/Agent")
```

```MATLAB
% train agent..............
maxEpisodes = 1000;
maxSteps = ceil(Tfinal/sampleTime);
trainOpts = rlTrainingOptions(...
    "MaxEpisodes",maxEpisodes, ...
    "MaxStepsPerEpisode",maxSteps, ...
    "ScoreAveragingWindowLength",50, ...
    "StopTrainingCriteria","AverageReward", ...
    "StopTrainingValue",20000, ...
    "Verbose", true, ...
    "Plots","training-progress");
trainOpts.SaveAgentCriteria = "EpisodeReward";
trainOpts.SaveAgentValue = 270;
```

```MATLAB
% doTraining...........................
doTraining = false; %false; %true; % Toggle this to true for training 

if doTraining
    load ('i01new.mat', 'Agent')
    % Train the agent.
    trainingStats = train(Agent,env,trainOpts);
    save ('i02new.mat','Agent');
else
    % Load pretrained agent for the example.
    rng(0);
    load ('i02new.mat', 'Agent')
    simOptions = rlSimulationOptions('MaxSteps',maxSteps);
    experience = sim(env,Agent,simOptions);
end
```
