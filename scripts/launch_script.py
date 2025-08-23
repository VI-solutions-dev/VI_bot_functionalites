#!/usr/bin/env python3
import os
import subprocess

def launch_ros_in_ard_ws():
    # 1. Point to your workspace folder
    workspace_dir = os.path.expanduser("~/ard_ws")

    # 2. The command you want to run inside that folder
    ros_cmd = "source devel/setup.bash && roslaunch robot_diff demarage.launch"

    # 3. Launch gnome-terminal in that directory, run the command, then keep it open
    subprocess.Popen([
        "gnome-terminal",
        "--working-directory", workspace_dir,
        "--",               # end of gnome-terminal args
        "bash", "-i", "-c", # start interactive bash
        f"{ros_cmd}; exec bash"
    ])

if __name__ == "__main__":
    launch_ros_in_ard_ws()
