#!/bin/bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="wlo1" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
export ROS_DOMAIN_ID=0
ros2 daemon stop && ros2 daemon start