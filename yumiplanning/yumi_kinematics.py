from tracikpy import TracIKSolver
from autolab_core import RigidTransform
import numpy as np

class YuMiKinematics():
    def __init__(self):
        '''
        Initializes the kinematics with the urdf file
        '''
        import os
        urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/../yumi_description/urdf/yumi.urdf"
        self.base_frame="base_link"
        self.l_tip_frame="gripper_l_base"
        self.r_tip_frame="gripper_r_base"
        '''taken from tracik:
        Speed: returns very quickly the first solution found
        % Distance: runs for the full timeout_in_secs, then returns the solution that minimizes SSE from the seed
        % Manipulation1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T)) (the product of the singular values of the Jacobian)
        % Manipulation2: runs for full timeout, returns solution that minimizes the ratio of min to max singular values of the Jacobian.
'''
        self.left_solver  = TracIKSolver(urdf_path,self.base_frame,self.l_tip_frame,timeout=.05,solve_type="Manipulation1")
        self.right_solver = TracIKSolver(urdf_path,self.base_frame,self.r_tip_frame,timeout=.05,solve_type="Manipulation1")
       
    def ik(self, left_pose=None, right_pose=None, left_qinit=None, right_qinit=None):
        '''
        given left and/or right target poses, calculates the joint angles and returns them as a tuple
        poses are RigidTransforms, qinits are np arrays
        '''
        lres=None
        rres=None
        if left_pose is not None:
            lres=self.left_solver.ik(left_pose.matrix,left_qinit)
        if right_pose is not None:
            rres=self.right_solver.ik(right_pose.matrix,right_qinit)
        return (lres,rres)

    def fk(self, qleft=None, qright=None):
        '''
        computes the forward kinematics for left and right arms. 
        qleft,qright are np arrays 
        returns a tuple of RigidTransform
        '''
        lres=None
        rres=None
        if qleft is not None:
            lpos=self.left_solver.fk(qleft)
            lres=RigidTransform(translation=lpos[:3,3],rotation=lpos[:3,:3],from_frame=self.l_tip_frame,to_frame=self.base_frame)
        if qright is not None:
            rpos=self.right_solver.fk(qright)
            rres=RigidTransform(translation=rpos[:3,3],rotation=rpos[:3,:3],from_frame=self.r_tip_frame,to_frame=self.base_frame)
        return (lres,rres)

    def compute_cartesian_path(self,l_start=None,l_goal=None,l_qinit=None,r_start=None,r_goal=None,r_qinit=None,
            N=10,jump_thresh=.5):
        '''
        Returns a traj interpolated linearly in cartesian space along the path between start and end
        all poses should be RigidTransform objects
        N is the mnumber of points to use when interpolating
        '''
        lres=None
        rres=None
        if(l_start is not None and l_goal is not None):
           waypoints = l_start.linear_trajectory_to(l_goal,N)
           matrices  = [t.matrix for t in waypoints]
           joint_traj = []
           past_state=l_qinit
           for m in matrices:
               past_state = self.left_solver.ik(m,past_state)
               joint_traj.append(past_state)
           if self.check_jumps(joint_traj,jump_thresh):
               print("Warning: jump detected in left traj")
           lres = joint_traj
        if(r_start is not None and r_goal is not None):
            waypoints = r_start.linear_trajectory_to(r_goal,N)
            matrices = [t.matrix for t in waypoints]
            joint_traj = []
            past_state = r_qinit
            for m in matrices:
                past_state = self.right_solver.ik(m,past_state)
                joint_traj.append(past_state)
            if self.check_jumps(joint_traj,jump_thresh):
               print("Warning: jump detected in right traj")
            rres=joint_traj
        return (lres,rres)

    def check_jumps(self,joint_traj,jump_thresh):
        '''
        this function takes a joint traj and returns the first index where there is a jump, or None if there are none 
        '''
        if len(joint_traj)<2:return
        tmp = joint_traj[0]
        jmps=False
        for i in range(1,len(joint_traj)): 
            diff=np.max(abs(joint_traj[i]-tmp))
            if diff>jump_thresh:
                print(diff)
                print(joint_traj[i])
                print(tmp)
                jmps=True
            tmp=joint_traj[i]
        return jmps

if __name__=="__main__":
    yk = YuMiKinematics()
    HOME_POSE = RigidTransform(translation=(.45,.207,.233),rotation=[0,1,0,0],from_frame='gripper_l_base',to_frame='base_link')
    left = RigidTransform(translation=(.5,-.1,.1),rotation=[0,1,0,0],from_frame='gripper',to_frame='world')
    reoriented = RigidTransform(translation=(.5,-.1,.1),rotation=[0.0018,-.85,-.464,-.249],from_frame='gripper',to_frame='world')
    start_state=np.deg2rad(np.array([-165,-100,50,-120,137,-114,-10]))
    lpath, rpath = yk.compute_cartesian_path(HOME_POSE,reoriented,l_qinit=start_state)
