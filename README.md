![jpg](/docs/img/cessna152.jpg "Our test superplane :)")
# General Information 🌍
This project contains one simple example using [Aerosandbox](https://github.com/peterdsharpe/AeroSandbox) (a Python package that helps you design and optimize aircraft and other engineered systems) lib for Python 3. The example's main goal is to find an optimal gliding trajectory for a small plane with the engine failure.
The maximum gliding distance depends on plane's aerodynamic parameters, initial altitude and initial velocity. To find it this code uses iPOPT optimal problem solver under the hood (IPOPT included into AeroSandbox). You can find all the mathematical explanation in the [AeroSandbox official example](https://github.com/peterdsharpe/AeroSandbox/blob/fb7e481b0dd251638e07c4379057b530b83367ea/tutorial/03%20-%20Trajectory%20Optimization%20and%20Optimal%20Control/03%20-%20The%20AeroSandbox%20Dynamics%20Stack/04%20-%202D%20Aircraft%20Dynamics%20for%20Mission%20Performance%20Analysis.ipynb). 
The input parameters are: ```h_0``` - the initial altitude, and ```v_0```, which is initial velocity. All of them  can be set in the [params.yaml config file](/src/aerosandbox_cessna/config/params.yaml). The result is states of flight (current altitude, velocity and ground path). All the calculations defined in [util_aerosandbox_cessna.py file](/src/aerosandbox_cessna/aerosandbox_cessna/util_aerosandbox_cessna.py).
Another important file is [aerosandbox_cessna.py file](/src/aerosandbox_cessna/aerosandbox_cessna/aerosandbox_cessna.py), which contains ROS 2 node  - it's used to call simulation function, get the results and publish it via ROS topics. You also can set publishing frequency (```publish_freq```) in the [params.yaml config file](/src/aerosandbox_cessna/config/params.yaml) (more info about this file below).


# Installation 🛫
1. Docker engine. This project runs inside Docker container, and requires Docker Engine/Docker Desktop. Follow the instructions on [Docker official website](https://www.docker.com/get-started/).
2. To use Docker inside VS Code several extensions are required. Install [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) and [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) extensions from Extensions tab on your left control panel.
3. Clone the repository:
```bash 
git clone git@github.com:citros-garden/aerosandbox_cessna.git
```

# Build 🛰
1. Open project root folder in VS Code.
2. Navigate to the lower-left corner of VS Code window and click on green mark.
3. Select "Reopen in container" option in the list on the top of the VS Code window. Wait a minute while Docker container is starting.
4. Open ```/src/aerosandbox_cessna/config/params.xml``` file to set parameters for simulation or just keep it default. Don't forget to save your changes!
5. Build ROS2 environment:
```bash 
colcon build
```
6. Source your environment:
```bash 
source install/local_setup.bash
```

# Preparing FoxGlove Studio 🪄
FoxGlove Studio is a robotics visualization and debugging tool, which can connect to ROS topic and get the data publishing through it. We will use it to visualizate the results of our simulations.

First of all, you need to download it from the [official website](https://foxglove.dev/) and install following the instructions. 

Next step is connecting to your ROS node. To perform it, open FoxGlove Studio and select *__Open connection__* option, then select *__Rosbridge__* option. Check the *__WebSocket URL__* field on the right of the window, it should contain ```ws://localhost:9090```. Now we are almost ready to go!

Last step is configuring the layout of FoxGlove. Navigate to the 'Add panel' on the top panel, and choose __*Plot*__ option. Now it's necessary to set Message paths in FoxGlove in advance. There are two ways to do it: using prepared layout file or manualy.

<details>
  <summary>To use prepared layout</summary>

1. Click on the top left button (with FoxGlove icon), then click on the *__view__* and choose *__Import layout from file...__*.

2. Copy code from file in [foxglove_layouts](/foxglove_layouts/) folder for the example you want to use.

3. Contragulations! You are ready to start!
</details>
<br>

OR 

<details>
  <summary>Manual plot layout configuration</summary>

Add two plot tabs to your layout, then go to the first plot's settings tab, 'Series' tab and add ```/aerosandbox_cessna/state.data[1]``` line to the Message path. Now go to the 'X Axis' tab, set 'Value' to the 'Path (accumulated)' and set the Message path to the ```/aerosandbox_cessna/state.data[0]```. This will give you an Altitude/Ground path plot.
To get the velocity results go the second plot settings and set Message path equals to ```/aerosandbox_cessna/state.data[2]```.
Although the best way to process simulation results is CITROS notebook.
</details>
<br>


# Run 🚀
1. Go back to the VS Code.
2. Prepare your FoxGlove studio (previous step, if you haven't done it yet).
3. Launch selected ROS2 package:
```bash 
ros2 launch aerosandbox_cessna launch.py
```
4. Watch the FoxGlove plot built from results!

# Citros usage 🛸
Although you can get simulation results using FoxGlove, the best way to work with such simulations and process the results is Citros! With its power, it is possible to create complex data processing scenarios, including the construction of more complex graphs, mathematical analysis and other high-level processing methods.

## Citros integration
1. Build Docker image:
```bash
docker build -t aerosandbox_cessna .
# OR *** when building from MAC M1 chip add FROM --platform=linux/amd64 ***
docker buildx build --platform linux/amd64 -t aerosandbox_cessna .   
```
2. Login to Citros
 ```bash
citros login
citros docker-login
```
3. Tag your project and push it into Citros cloud
 ```bash
docker tag aerosandbox_cessna us-central1-docker.pkg.dev/citros/lulav/aerosandbox_cessna
docker push us-central1-docker.pkg.dev/citros/lulav/aerosandbox_cessna
```


# Extras

![png](/docs/img/img0.png "Results example")
![png](/docs/img/img1.png "Results example")
