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