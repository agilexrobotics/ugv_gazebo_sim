# AgileX Product Gazebo Simulate



## Install the Gazebo software

Gazebo is  a simulator. Gazebo simulates multiple robots in a 3D environment, with extensive dynamic interaction between objects.

[http://gazebosim.org](http://gazebosim.org/)

Download and install gazebo you can go to the website :http://gazebosim.org/install

------



## Current support Gazebo simulation product list

| Product name      | support status | Link                                                         |
| :---------------- | -------------- | ------------------------------------------------------------ |
| SCOUT 1.0×        | ×              |                                                              |
| HUNTER 1.0        | ×              |                                                              |
| SCOUT 2.0         | √              | [Scout Series ](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/scout) |
| HUNTER 2.0        | √              | [Hunter 2.0](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/hunter) |
| HUNTER SE         | √              | [HUNTER SE](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/hunter_se) |
| SCOUT MINI        | √              | [Scout Series ](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/scout) |
| SCOUT MINI(OMNI)  | ×              |                                                              |
| TRACER            | √              | [Tracer](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/tracer) |
| TRACER MINI MODLE | √              | [TRACER MINI MODLE](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/tracer_mini) |
| BUNKER            | √              | [BUNKER](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/bunker) |
| BUNKER MINI       | ×              |                                                              |
| BUNKER PRO        | ×              |                                                              |
| AUTOKI MODLE      | √              | [AUTOKI](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/autokit) |
| RANGER MINI V1    | √              | [Ranger Mni Series](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/ranger_mini) |
| RANGER MINI V2    | √              | [Ranger Mni Series](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/ranger_mini) |
| RANGER MINI V3    | √              | [Ranger Mni Series](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/ranger_mini) |
| LIMO              | √              | [Limo](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/limo) |
| LIMO COBOT        | √              | [LIMO COBOT](https://github.com/agilexrobotics/limo_cobot_sim) |
| SCOUT COBOT KIT   | √              | [SCOUT COBOT KIT](https://github.com/agilexrobotics/scout_cobot_sim) |
| COBOT S KIT       | √              | [COBOT S KIT](https://github.com/agilexrobotics/cobot_s_sim) |



## About usage

1) clone the current repositories to your own workspace
2) Go to you use product
3) Each independent chassis product has its own independent instructions in the corresponding file directory

### Ranger mini v2 & Ranger mini v3

Because some models are too large to be uploaded to github, they are compressed and need to be extracted after downloading

Ranger mini v2

``` bash
cd ranger_mini/ranger_mini_v2/meshes/
unzip ranger_base.zip
```

Ranger mini v3

```
cd ranger_mini/ranger_mini_v3/meshes/
unzip ranger_base.zip
```

