#!/usr/bin/env python3
import os, subprocess, rospy
from std_srvs.srv import Trigger, TriggerResponse

GNOME = "/usr/bin/gnome-terminal"           # full path is safest
WORK  = os.path.expanduser("~/ard_ws")      # your catkin ws
CMD   = "source devel/setup.bash && roslaunch robot_diff demarage.launch"

def handle(_req):
    try:
        subprocess.Popen([
            GNOME,
            "--working-directory", WORK,
            "--",
            "bash", "-i", "-c", f"{CMD}; exec bash"
        ], env=dict(os.environ, DISPLAY=":0"))   # DISPLAY is needed if node is launched from a non-GUI shell
        return TriggerResponse(success=True, message="Terminal launched")
    except Exception as e:
        return TriggerResponse(success=False, message=str(e))

if __name__ == "__main__":
    rospy.init_node("run_script_server")
    rospy.Service("run_script", Trigger, handle)
    rospy.loginfo("Service /run_script ready")
    rospy.spin()

# import rospy
# from std_srvs.srv import Trigger, TriggerResponse
# import subprocess, os

# def handle_run(req):
#     # 4️⃣ Here’s where you point at your real script
#     launcher = "/home/boucherikator/ard_ws/src/robot_diff/scripts/launch_script.py"

#     # 5️⃣ Execute it
#     res = subprocess.run(
#         ['python3', launcher],
#         stdout=subprocess.PIPE,
#         stderr=subprocess.PIPE,
#         text=True
#     )

#     # 6️⃣ Return its stdout/stderr back to the caller
#     return TriggerResponse(
#         success=(res.returncode == 0),
#         message=res.stdout or res.stderr
#     )

# if __name__ == '__main__':
#     rospy.init_node('run_script_server')
#     s = rospy.Service('run_script', Trigger, handle_run)
#     rospy.spin()
