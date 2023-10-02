---
slug: Aerosandbox example using CITROS
title: Aerosandbox example using CITROS
authors: [gtep]
tags: [CITROS]
---

Blog posts support [Docusaurus Markdown features](https://docusaurus.io/docs/markdown-features), such as [MDX](https://mdxjs.com/).

![jpg](img/cessna152.jpg "https://en.wikipedia.org/wiki/File:Cessna_152_PR-EJQ_(8476096843).jpg")

## Example overview 🌐 

The project is a simulation tool developed using ROS (Robot Operating System) nodes and leverages the Aerosandbox Python library for aerodynamic calculations. Its primary objective is to simulate gliding flight scenarios for a Cessna 152 aircraft in the event of engine failure.

The user provides flight parameters as input parameters to configure the simulation. These parameters are essential for defining the initial conditions of the simulated flight.

Once the simulation is initiated, the ROS nodes orchestrate the execution. The simulation takes into account various flight dynamics and aerodynamic factors to model the gliding behavior of the Cessna 152. The maximum gliding distance depends on plane's aerodynamic parameters, initial altitude and initial velocity. To find it this code uses iPOPT optimal problem solver under the hood (IPOPT included into AeroSandbox).

The output of the simulation comprises critical flight data, such as altitude, velocity, and other relevant parameters, recorded over time intervals. These results are published via ROS topics, allowing for real-time data visualization, analysis, and integration with other ROS-based systems.

You can find more information about this useful aerodynamics library on [Aerosandbox official website](https://github.com/peterdsharpe/AeroSandbox).

## Code overview
This example consists of several files:
1. ```aerosandbox_cessna.py```. This the main project's file with a ROS 2 node defined. The ROS 2 node is used to orchestrate flight simulations of a Cessna aircraft and publishes the results via ROS topics. Users provide input parameters such as initial altitude (```h_0```) and initial velocity (```v_0```), crucial for defining the starting conditions of the simulation. The code invokes a simulation function (*_util_aerosandbox_cessna.run_*) with the specified parameters to perform the Cessna aircraft flight dynamics simulation. A timer is set up to periodically publish the aircraft's state data, including its position, altitude, and velocity. These state updates are logged and published on the ROS topic */aerosandbox_cessna/state*. Upon completing the simulation, the ROS 2 runtime is shut down.

2. ```util_aerosandbox_cessna.py```. This Python code is a optimization tool for modeling the flight dynamics of an aircraft. The code utilizes the Aerosandbox library and initializes a simulation to study the behavior of a Cessna 152 (this aircraft type is used for simplicity) aircraft in flight. The simulation is constructed using optimization techniques, and it defines a time horizon with an initial guess for the simulation duration. The dynamics of the aircraft, including its position, altitude, speed, and angle of attack, are modeled using differential equations. Aerodynamic forces and gravitational effects are incorporated into the simulation to simulate real-world conditions. The optimization problem aims to maximize the downrange distance traveled by the aircraft. You can find all the mathematical explanation in the [AeroSandbox example page](https://github.com/peterdsharpe/AeroSandbox/blob/fb7e481b0dd251638e07c4379057b530b83367ea/tutorial/03%20-%20Trajectory%20Optimization%20and%20Optimal%20Control/03%20-%20The%20AeroSandbox%20Dynamics%20Stack/04%20-%202D%20Aircraft%20Dynamics%20for%20Mission%20Performance%20Analysis.ipynb). 
Upon solving the optimization problem, the simulation results are logged, and the code returns the aircraft's position, altitude, and speed data to the main ROS node. 
 
3. ```cessna152.py```. The plane's aerodynamic functions defined here: 
    The code defines two airfoil profiles, "naca2412" and "naca0012," and generates their aerodynamic polars. These profiles are used to define the wing sections of the aircraft.
    An "Airplane" object consists of several components, including wings, horizontal stabilizers, vertical stabilizers, and a fuselage. The wings are defined with multiple wing cross-sections ("WingXSec") that specify the airfoil shape, chord length, and other wing parameters. The wing geometry is defined symmetrically. The horizontal stabilizer and vertical stabilizer components are also defined with their respective cross-sections and geometry. The fuselage is defined with a series of cross-sections that describe its shape and dimensions. 
    The code then draws a three-view representation of the Cessna 152 using the defined geometry.

The project also contains several additional files for aerodynamic calculations under the ```resource``` folder: ```naca0012.json``` and ```naca2412.json```. They store aerodynamical profiles used for wings and other aerofoil planes for the Cessna 152. They are included into ```cessna152.py``` file.

## Local Usage 💻

All project installation and usage information also available in the project [GitHub page](https://github.com/citros-garden/aerosandbox_cessna).

### Installation 🔨
1. Docker engine. This project runs inside Docker container, and requires Docker Engine/Docker Desktop. Follow the instructions on [Docker official website](https://www.docker.com/get-started/).
2. To use Docker inside VS Code several extensions are required. Install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) and [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) extensions from Extensions tab on your left control panel.
3. Clone the repository:
```bash 
git clone git@github.com:citros-garden/aerosandbox_cessna.git
```

### Build 🏠
1. Open project root folder in VS Code.
2. Navigate to the lower-left corner of VS Code window and click on green mark.
3. Select "Reopen in container" option in the list on the top of the VS Code window. Wait a minute while Docker container is starting.
2. Open ```/src/aerosandbox_cessna/config/params.xml``` file to set parameters for simulation or just keep it default. Don't forget to save your changes!
3. Build ROS 2 environment:
```bash 
colcon build
```
4. Source the environment:
```bash 
source install/local_setup.bash
```

### Preparing FoxGlove Studio 🪄
FoxGlove Studio is a robotics visualization and debugging tool, which can connect to ROS topic and get the data publishing through it. We will use it to visualizate the results of our simulations.

First of all, you need to download it from the [official website](https://foxglove.dev/) and install following the instructions. 

Next step is connecting to your ROS node. To perform it, open FoxGlove Studio and select *__Open connection__* option, then select *__Rosbridge__* option. Check the *__WebSocket URL__* field on the right of the window, it should contain ```ws://localhost:9090```. Now we are almost ready to go!


OR

:::tip

You can use prepared layout: Go to the ```Layout``` tab on the top panel, then click on import_layout button and select the file from foxglove_layouts folder.

:::

![gif](img/gif0.gif "FoxGlove example")

:::tip

The best way to process simulation results is CITROS notebook 🍋 :)

:::


### Run 🚀
1. Go back to the VS Code.
2. Launch ROS 2 package:
```bash 
ros2 launch aerosandbox_cessna aerosandbox_cessna.launch.py
```
3. Watch the FoxGlove plot built from results!

OR

:::tip

You can use Visual Code Tasks: simply press ```Alt+T``` and select the task you want to build. This will build, source and launch the project automaticly.

:::

![png](img/img0.png "FoxGlove example")

## CITROS usage 🛸
Although you can get simulation results using FoxGlove, the best way to work with such simulations and process the results is CITROS! With its power, it is possible to create complex data processing scenarios, including the construction of more complex graphs, mathematical analysis and other high-level processing methods.

### CITROS installation 🛫

First of all, to use all the powerfull CITROS features usage requires CITROS installation: follow the instructions on the CITROS CLI [GitHub page](https://github.com/lulav/citros_cli).

### Configuring the project ⚙️
After all the prerequisites done, we can start configuring our project. The starting point is the Lunar_Starship devcontainer loaded and running, CITROS CLI is installed and ready.
1. Initialize CITROS:
```bash 
citros init
```
Now you can see ```.citros``` folder in the explorer.

2. Configuring the setup. We need to set up the maximum perfomance available: timeout, CPU, GPU and Memory. To perform it, we need to define it in the ```.citros/simulations/simulation_aerosandbox_cessna.json```. The recommended setup is minimum 180 seconds timeout, 2 CPU, 2 GPU and 1024 MB of Memory. Don't forget to save the file!

3. Configuring the params setup. You can find default setup in ```.citros/parameter_setups/default_param_setup.json```. The Aerosanbox simulation has the following ROS parameters:

$$
\begin{array}{|c|c|}
\hline
\text{Parameter} & \text{Description} \\
\hline
h\_0 & \text{Initial alitude (m)} \\
v\_0 & \text{Initial velocity (knots)} \\
publish~freq & \text{frequency of publishing} \\
\hline
\end{array}
$$


Don't forget to save the file!

### Syncing the project's setup 📡
Now we can sync our project settings with CITROS server:
```bash 
citros commit
citros push
```
### Running locally 🛋️
Since all the preparations done, we can launch it locally (your project should be built and sourced before that, check the instructions above):
```bash 
citros run -n 'aerosandbox_cessna' -m 'local test run'
```
Select the launch file (should be the only one here) by pressing ```Enter``` button and wait for the output in the terminal. To plot the local run results you can use FoxGlove.

### Uploading Docker image to the CITROS database and running in the cloud 🛰️
1. We need to build and push Docker container image to the CITROS server:
```bash 
citros docker-build-push
```

2. Finally, we can run it in the cloud! Simply add ```-r``` to the terminal command: 
```bash 
citros run -n 'aerosandbox_cessna' -m 'cloud test run' -r
```
Select the launch file (should be the only one here) by pressing ```Enter``` button. Now the simulation is running in the CITROS server, and it will upload results to the CITROS database automaticly.

### CITROS Web usage and data analysis 🌌
#### Launching project via CITROS Web
The best way to use all the innovative capabilities of CITROS is through it's Web interface. The following manual explains how to run this project in the cloud and how to process the simualtion results.
The starting point is CITROS main page, user is logged in and the project Docker image is built and pushed to the cloud (see the [manual](#uploading-docker-image-to-the-citros-database-and-running-in-the-cloud-🛰️) above).
1. Go to the ```Repositories``` page clicking on the tab on the top;
2. Find your project and open it;
3. Navigate to the ```Runs``` tab;
4. Click on the ```Run Simulation``` button on the right;
5. Now you can choose the project and the simulation setup from the droplists, set the number of repeats and how many simulations should run in parallel, type the Name of the run and the additional message. This window also shows the perfomance preset.
6. We are ready to go! Start the Batch with the button below.

The simualtion launched! Open the Run you just started in the list on ```Runs``` page to check how is it going. In this page you can find all the runs of this batch. The number of runs here equals to the number of runs you've set before.
Navigate to the Run by clicking on it in the table:
* The main part of this page is a simulation's log. Here you can find all the logging information from all levels: from your code logs up to the CITROS system information.
* The right part of the page provides additional information about Events: the main stages of the simulation run.


#### Working with integrated Jupiter Notebooks
CITROS Web provides powerfull data analisys package, which is comprehensive solution for data query, analysis and visualization. With its extensive features, you can quickly and easily extract valuable insights from your data. To use it, the Jupiter Notebook support is built-in. 
Navigate to our project ```Code``` page, open the Notebooks folder and click on the notebook file. Here you can see the usual Jupiter editor's interface: you can add blocks of code or built-in Markdown engine, run and save notebook and control the Python kernel.

You can find all the data analisys package [here](https://citros.io/doc/docs_data_analysis).


## Extras
### FoxGlove examples

![png](img/img1.png "FoxGlove example")
