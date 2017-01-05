# ros-drone-orchestrator


## Run orchestrator
# Start the map server
```bash
rosrun map_server map_server src/orchestrator/maps/map.yaml
```

# Transform the map as a static map
```bash
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map fixed_map 100
```

# Run orchestrator itself
```bash
rosrun orchestrator orchestrator.py
```

## Run rviz
```bash
rosrun rviz rviz src/orchestrator/maps/rviz_config.rviz
```
Then you can click on 'Publish point' to manually move the drone


## Paul

rosrun orchestrator orchestrator.py 
git push https://github.com/Amoki/bebop-safe-control.git master
rostopic pub -1 /orchestrator/order geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

