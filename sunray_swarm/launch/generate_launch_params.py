#!/usr/bin/env python3
# -*-coding:utf-8-*-
import logging, time, sys, getopt
import robomaster
from robomaster import robot
from multi_robomaster import tool
import re
SN_LIST = [
    "0TQZM47CNT046P", "0TQZM48CNT06PF", "0TQZM48CNT06PY",
    "0TQZJADCNT18QU", "0TQZM43CNT03UU", "0TQZM43CNT03FC",
    "0TQZM43CNT03RR", "0TQZJADCNT1A6Z"
]
local_ip = "192.168.25.59"

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

    # 创建或打开要写入的YAML文件
    with open("drone_params.yaml", "w") as yaml_file:
        yaml_file.write("drones:\n")
        for idx, sn in enumerate(SN_LIST):
            if sn in robot_sn_dict:
                ip, port = robot_sn_dict[sn]
                yaml_file.write(f"  rmtt_{idx+1}:\n")
                yaml_file.write(f"    uav_id: {idx+1}\n")
                yaml_file.write(f"    uav_ip: {ip}\n")
                yaml_file.write(f"    local_ip: {local_ip}\n")
                yaml_file.write(f"    local_port: {8890 + idx}\n")
                yaml_file.write(f"    video_port: {11111 + idx}\n")
                yaml_file.write(f"    enable_camera: true\n")
                yaml_file.write(f"    sn: {sn}\n")

    print("YAML file 'drone_params.yaml' has been created.")