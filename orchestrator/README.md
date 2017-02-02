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


Docs
-------------------------

eenable nelnet on bebop: 4 short press on on/off button

enable write on telnet: mount -o remount,rw /


on the bebop, `ls /bin/onoffbutton`
result: `longpress_0.sh      shortpress_1.sh     shortpress_4.sh     verylongpress_0.sh`
You will see some files names as X_Y.sh
Where X is a button pressure time (short, long or verylong) and Y is the number of time you need to press the button to run the script
For example: when you want to shutdown the drone, you have to short press the button once. It procs the shortpress_1.sh script that shutdown the drone
the script shortpress_4.sh enable telnet
the script verylongpress_0.sh reset the drone

I added the script longpress_0.sh that setup the wifi as client instead of access point
This is the script:
```sh
ESSID=bebop-hotspot
IP=192.168.2.129
NETMASK=255.255.255.0
GATEWAY=192.168.2.1
DEFAULT_WIFI_SETUP=/sbin/broadcom_setup.sh

# Set light to orange
BLDC_Test_Bench -G 1 1 0 >/dev/null

# Check whether drone is in access point mode
if [ $(bcmwl ap) -eq 1 ]
then
        echo "Trying to connect to $ESSID" | logger -s -t "LongPress" -p user.info

        # Bring access point mode down
        $DEFAULT_WIFI_SETUP remove_net_interface
        # Configure wifi to connect to given essid
        ifconfig eth0 down
        bcmwl down
        bcmwl band auto
        bcmwl autocountry 1
        bcmwl up
        bcmwl ap 0
        bcmwl join ${ESSID}
        # setup manual ip 
        ifconfig eth0 ${IP} netmask ${NETMASK} up
        route add default gw ${GATEWAY}
else
        # Should make drone an access point again
        # Bug: does not work yet (turn drone off & on instead)
        $DEFAULT_WIFI_SETUP create_net_interface
fi

# Set light back to green then orange then red then green again
sleep 1; BLDC_Test_Bench -G 0 1 0 >/dev/null
sleep 1; BLDC_Test_Bench -G 1 1 0 >/dev/null
sleep 1; BLDC_Test_Bench -G 1 0 0 >/dev/null
sleep 1; BLDC_Test_Bench -G 0 1 0 >/dev/null

```

With this config, the drone will start its access point on boot, but swap to client mode when you long press the button for about 5 seconds
