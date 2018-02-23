#!/bin/bash
WD=../../../devel/lib/industry_camera_bridge
sudo chown root:root "$WD/camera_publisher"
sudo chmod a+rx "$WD/camera_publisher"
sudo chmod u+s "$WD/camera_publisher"
