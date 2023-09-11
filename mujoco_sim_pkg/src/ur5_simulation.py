"""
Mujoco Ros Simulation for UR5e by METU ROMER Kovan Lab for Kalfa Project
More information : https://metu-kalfa.github.io/
Code is handled by Yunus Talha Erzurumlu
e-mail : yunus.erzurumlu@metu.edu.tr
2023
"""

import mujoco
from mujoco import viewer
import numpy as np

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class UR5Simulator:
    
    def __init__(self, path) -> None:
        
        self.model = mujoco.MjModel.from_xml_path(path) 
        self.data =  mujoco.MjData(self.model)
        self.data.qpos[:] = np.array([-0.21493, -1.92668, -0.521474, -0.0147683, 0.0013998, 3.80268e-05])

        self.joint_state_pub = rospy.Publisher('/mujoco_joint_state', JointState, queue_size=10)
        self.joint_state_pub_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.mujoco_commands_sub = rospy.Subscriber('/ur5e_hardware_interface/mujoco_commands', JointState, self.mujoco_commands_callback)


    def view(self):
 
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        return self.viewer
 
    def step(self):
        

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        joint_state.position = self.get_joint_pos()
        joint_state.velocity = self.get_joint_vels()
        joint_state.effort = self.get_joint_torques()
        self.joint_state_pub.publish(joint_state)
        self.joint_state_pub_.publish(joint_state)
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()

    def get_joint_pos(self):
        
        joint_poisitons = np.asarray(self.data.qpos, dtype=np.double)
        return joint_poisitons
    
    def get_joint_vels(self):

        joint_vels = np.asarray(self.data.qvel, dtype=np.double)
        return joint_vels
    
    def get_joint_torques(self):

        joint_torques = np.asarray(self.data.qfrc_actuator, dtype=np.double)
        return joint_torques
    
    def control(self, torques):
        self.data.ctrl[:] = torques
        # self.data.ctrl[torques == 0] = np.nan  # Clear zero values

    def mujoco_commands_callback(self, msg):

        torq = np.array(msg.position)
        #torq = np.array([torq[0], torq[1],0,0,0,0])
        self.control(torq)

if __name__ == "__main__":
    rospy.init_node("mujoco_sim_node")
    sim = UR5Simulator('src/mujoco_sim_pkg/src/universal_robots_ur5e/ur5e.xml')
    viewer = sim.view()
    rate = rospy.Rate(60)
    while not rospy.is_shutdown() and viewer.is_running():  
        # print("Joint Positions : ", sim.get_joint_pos().dtype)
        # print("Joint Velocities : ", sim.get_joint_vels().dtype)        
        #sim.control([-50,-25,-30,20,0,0])
        sim.step()
        rate.sleep()