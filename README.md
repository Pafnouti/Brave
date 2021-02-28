# Brave

## Dependencies
The web interface requires [Node.js](https://nodejs.org/en/) to be installed. You can check [this website](https://nodejs.org/en/download/) to install Node.js.

If the installation have been successful, you should be able to launch the following command :
```shell
 $ node -v
 # We installed v12.20.1
 ```

In order to install the packages needed for the interface, navigate to */workspaceRos/src/nodejsTelemetry* and launch this command :
```shell
 $ npm install
 ```

 Python dependencies are :
- numpy
- scipy
- shapely
- pandas

They can be pip-installed.

 ## Launch the nodes
 The folder workspaceRos/src/navigator_node/launch/ contains our entry points.
  ```shell
 $ roslaunch navigator_node main.launch
 ```

 | Launch file        | Usage           |
| ------------- |:-------------:|
|  **main.launch**     | This file will launch all the nodes required to run on the sailboat. |
| **sim.launch**      | This file will launch all the nodes required to run a simulated sailboat.      |  
| **debug.launch** | This file will launch only the interface server.      |   

## Connect to Brave 

### via Ethernet
 ```shell
 $ ssh brave@10.42.0.1
 #brave's password : brave
 ```
### via SSH
 ```shell
 $ ssh brave@192.168.0.55
 #brave's password : brave
 ```
### Connect to the interface
Just open <http://192.168.0.55:3000> in a web browser on a device connected to the wifi network after launching the *nodejsTelemetry* node (or a launch file).


 ## Router Node
 A Router is an algorithm that find the best trajectory for a sailboat. 
 There are many ways to do it, one of which uses isochrones. 
 The inputs are the boat polar, the wind at each point and at each time, obstacles and safe zones.
 The algorithm determines where the sailboat can be at a given times.

 To launch the router alone :
 ```shell
 $ python workspaceRos/src/router/src/router.py
 ```

 To launch the node :
 ```shell
 $ roslaunch router router.launch
 ```
