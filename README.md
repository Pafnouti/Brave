# Brave

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

 ## Launch the nodes
 The folder workspaceRos/src/navigator_node/launch/ contains our entry points.
  ```shell
 $ roslaunch navigator_node main.launch
 ```
 
 **main.launch**
 
 This file will launch all the nodes required to run on the sailboat.
 
 **sim.launch**
 
 This file will launch all the nodes required to run a simulated sailboat.
 
  **debug.launch**
  
 This file will launch only the interface server.
