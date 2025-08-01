#!/usr/bin/env python3
# -*-coding:utf-8-*-
# Copyright (c) 2021 Tianbot.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging, time, sys, getopt
import robomaster
from robomaster import robot
from multi_robomaster import tool
import re

# RMTT无人机SN号列表，几台无人机就填写几个
SN_LIST = ["0TQZM47CNT046P", "0TQZM41CNT02AY", "", "", "", "", "", ""]
# local IP指开发主机的IP？
local_ip = "192.168.25.91"

if __name__ == '__main__':
    help_str ='rmtt_scan_ip.py -n <num of drones>' 
    try:
        opts, args = getopt.getopt(sys.argv[1:], "h:n:", ["help", "num=1"])
    except getopt.GetoptError:
        print(help_str)
        sys.exit(2)

    if not args:
        num = 1
        print('Searching for one drone...')
    elif not opts:
        print('Please use the following cmd to specify the number of drones: \n',
        '\033[95m'+help_str+'\033[0m')
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            print(help_str)
            sys.exit()
        elif opt in ("-n", "--num"):
            num = int(arg)
        else:
            print(help_str)
            sys.exit(2)
    #robomaster.enable_logging_to_file()
    logger_name = "multi_robot"
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.DEBUG)

    robot_host_list={}
    robot_sn_dict={}

    client = tool.TelloClient()
    client._conn.local_ip = '0.0.0.0'
    client.start()

    robot_host_list=client.scan_multi_robot(num)

    for host in robot_host_list:
        proto = tool.TelloProtocol("sn?", host)
        client.send(proto)
        logger.info("send cmd")

    cur_time = time.time()
    while client.queue.qsize() < 1:
        if time.time() - cur_time > 10:
            raise Exception("get sn timeout")

    while not client.queue.empty():
        proto = client.queue.get()
        if proto.text is None:
            raise Exception("recv data is None")
        robot_sn_dict[proto.text] = proto.host
        time.sleep(0.1)
        logger.info("get host")

    client.close()

    # 生成rmtt_all_drone.launch脚本
    with open("rmtt_all_drone.launch","w") as f:
        f.write('<?xml version="1.0"?>\n')
        f.write("<launch>\n")
        f.write(f'\t<arg name="local_ip" default="{local_ip}" />')
        f.write('\n')
        
        for idx,sn  in enumerate(SN_LIST):
            if sn not in robot_sn_dict.keys():
                continue
            ip = robot_sn_dict[sn][0]
            port = robot_sn_dict[sn][1]
            f.write(f'\t<!-- 第{idx+1}台无人机 -->\n')
            f.write(f'\t<group ns="/sunray_swarm/rmtt_{idx +1}">\n')
            f.write(f'\t\t<!-- 启动rmtt_driver -->\n')
            f.write(f'\t\t<node pkg="rmtt_driver" type="rmtt_node.py" name="rmtt_driver" output="screen">\n')
            f.write(f'\t\t\t<param name="drone_sn" type="string" value="{sn}" />\n')
            f.write(f'\t\t\t<param name="drone_ip" type="string" value="{ip}" />\n')
            f.write(f'\t\t\t<param name="drone_port" type="string" value="{port}" />\n')
            f.write(f'\t\t\t<param name="local_ip" type="string" value="$(arg local_ip)" />\n')
            f.write(f'\t\t\t<param name="local_port" type="string" value="{8890+idx}" />\n')
            f.write(f'\t\t\t<param name="video_port" type="string" value="{11111+idx}" />\n')
            f.write(f'\t\t\t<param name="enable_camera" type="bool" value="false" />\n')
            f.write(f'\t\t</node>\n')
            f.write(f'\t\t<!-- 启动rmtt_control_node -->\n')
            f.write(f'\t\t<node pkg="sunray_swarm" type="rmtt_control_node" name="rmtt_control_node" output="screen">\n')
            f.write(f'\t\t\t<param name="agent_id" value="{idx+1}" />\n')
            f.write(f'\t\t\t<param name="agent_ip" value="{ip}" />\n')
            f.write(f'\t\t\t<rosparam command="load" file="$(find sunray_swarm)/launch_rmtt/rmtt_params.yaml" />\n')
            f.write(f'\t\t</node>\n')
            f.write(f'\t</group>\n')
            f.write('\n')


        f.write("</launch>\n")
       
    
