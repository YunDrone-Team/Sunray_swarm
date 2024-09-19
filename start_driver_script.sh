#!/bin/bash
# 启动turn.service服务
echo '123' | sudo -S systemctl start turn.service

# 打开一个新的终端窗口执行start_driver命令
gnome-terminal --window -e 'bash -c "systemctl start turn.service; exec bash"'
