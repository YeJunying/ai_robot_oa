<img src="img/header.jpg" alt="Header" width="100%"/>

The repository is meant for leveraging system development and robot deployment for ground-based autonomous navigation and exploration. Containing a variety of simulation environments, autonomous navigation modules such as collision avoidance, terrain traversability analysis, waypoint following, etc, and a set of visualization tools, users can develop autonomous navigation systems and later on port those systems onto real robots for deployment.

Please use instructions on our [project page](https://www.cmu-exploration.com).

# 避障

## 与ai_robot联合测试

```shell
# ai_robot中切换到sim_oa_falco分支
roslaunch ai_robot_core test_sim_oa.launch
```

```shell
# noetic-ai-robot分支
roslaunch vehicle_simulator ai_robot_sim_local_planner.launch
```

//
