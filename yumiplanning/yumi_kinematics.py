from tracikpy import TracIKSolver
from autolab_core import RigidTransform
import numpy as np

class YuMiKinematics():
    L_NICE_STATE=np.deg2rad(np.array([-71.52785377, -62.91241387, 61.09700753, 17.98294573,
             108.93368164,  75.65987325, 139.55185761]))
    R_NICE_STATE=np.array([ 1.21442839, -1.03205606, -1.10072738,  0.2987352,  
             -1.85257716,  1.25363652,-2.42181893])
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
        self.left_solvers={}
        self.left_solvers["Distance"]=TracIKSolver(urdf_path,self.base_frame,self.l_tip_frame,timeout=.05,solve_type="Distance")
        self.left_solvers["Manipulation1"] = TracIKSolver(urdf_path,self.base_frame,self.l_tip_frame,timeout=.05,solve_type="Manipulation1")
        self.right_solvers={}
        self.right_solvers["Distance"]= TracIKSolver(urdf_path,self.base_frame,self.r_tip_frame,timeout=.05,solve_type="Distance")
        self.right_solvers["Manipulation1"] = TracIKSolver(urdf_path,self.base_frame,self.r_tip_frame,timeout=.2,solve_type="Manipulation1")
       
    def ik(self, left_pose=None, right_pose=None, left_qinit=None, right_qinit=None, solve_type="Distance"):
        '''
        given left and/or right target poses, calculates the joint angles and returns them as a tuple
        poses are RigidTransforms, qinits are np arrays
        solve_type can be "Distance" or "Manipulation1" (See constructor for what these mean)
        '''
        lres=None
        rres=None
        if left_pose is not None:
            lres=self.left_solvers[solve_type].ik(left_pose.matrix,left_qinit)
        if right_pose is not None:
            rres=self.right_solvers[solve_type].ik(right_pose.matrix,right_qinit)
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
            lpos=self.left_solvers["Distance"].fk(qleft)
            lres=RigidTransform(translation=lpos[:3,3],rotation=lpos[:3,:3],from_frame=self.l_tip_frame,to_frame=self.base_frame)
        if qright is not None:
            rpos=self.right_solvers["Distance"].fk(qright)
            rres=RigidTransform(translation=rpos[:3,3],rotation=rpos[:3,:3],from_frame=self.r_tip_frame,to_frame=self.base_frame)
        return (lres,rres)

    def compute_cartesian_path(self,l_start=None,l_goal=None,l_qinit=None,r_start=None,r_goal=None,r_qinit=None,
            N=20,jump_thresh=.3):
        '''
        Returns a traj interpolated linearly in cartesian space along the path between start and end
        all poses should be RigidTransform objects
        N is the mnumber of points to use when interpolating
        If no qinit is specified, the solver won't necessarily produce a close solution, so there may be
        a large motion to get to the first waypoint in the path
        '''
        lres=None
        rres=None
        if(l_start is not None and l_goal is not None):
           waypoints = l_start.linear_trajectory_to(l_goal,N)
           joint_traj = self.compute_traj(self.left_solvers,waypoints,jump_thresh,l_qinit)
           if self.check_jumps(joint_traj,jump_thresh):
               print("Warning: jump detected in left traj")
           lres = joint_traj
        if(r_start is not None and r_goal is not None):
            waypoints = r_start.linear_trajectory_to(r_goal,N)
            joint_traj = self.compute_traj(self.right_solvers,waypoints,jump_thresh,r_qinit)
            if self.check_jumps(joint_traj,jump_thresh):
               print("Warning: jump detected in right traj")
            rres=joint_traj
        return (lres,rres)

    def compute_traj(self, solvers, waypoints, jump_thresh,qinit=None):
        '''
        Calculates the IK for each pose in waypoints, attempting to use Manipulation1 when possible and 
        defaulting to Distance when movement exceeds the jump_thresh
        solvers=either self.left_solvers or self.right_solvers
        waypoints=list of RigidTransform
        jump_thresh=describes the max allowed distance between joint configs
        '''
        joint_traj=[]
        q_manip=solvers["Manipulation1"].ik(waypoints[0].matrix, qinit)
        if qinit is None or np.max(np.abs(q_manip-qinit))>jump_thresh:
            joint_traj.append(solvers["Distance"].ik(waypoints[0].matrix, qinit))
        else:
            joint_traj.append(q_manip)
        #then do the rest of the traj using distance
        for i in range(1,len(waypoints)):
            m = waypoints[i].matrix
            joint_traj.append(solvers["Distance"].ik(m,joint_traj[-1]))
        return joint_traj

    def check_jumps(self,joint_traj,jump_thresh):
        '''
        this function takes a joint traj and returns whether there is a jump exceeding jump_thresh
        '''
        if len(joint_traj)<2:return
        tmp = joint_traj[0]
        jmps=False
        for i in range(1,len(joint_traj)): 
            diff=np.max(abs(joint_traj[i]-tmp))
            if diff>jump_thresh:
                print("Jump of magnitude",diff,"found")
                jmps=True
            tmp=joint_traj[i]
        return jmps
    
    '''
    For some ungodly reason, yumi doesn't format the joint data in order of physical joint
    the actual order is [1,2,4,5,6,7,3]
    meaning the 3rd physical joint appears last in the data array xD
    these functions convert between physical (urdf, used with IK) and yumi (for sending commands)
    '''
    @staticmethod
    def urdf_order_2_yumi(q):
        qnew=np.zeros(7)
        qnew[6]=q[2]
        qnew[0:2]=q[0:2]
        qnew[2:6]=q[3:7]
        return qnew

    @staticmethod
    def yumi_order_2_urdf(q):
        qnew=np.zeros(7)
        qnew[0:2]=q[0:2]
        qnew[2]=q[6]
        qnew[3:7]=q[2:6]
        return qnew

if __name__=="__main__":
    yk = YuMiKinematics()
    L_HOME_POSE = RigidTransform(translation=(.5,.2,.256),rotation=[0,1,0,0],from_frame='gripper_l_base',to_frame='base_link')
    R_HOME_POSE = RigidTransform(translation=(.5,-.2,.256),rotation=[0,1,0,0],from_frame='gripper_l_base',to_frame='base_link')
    left = RigidTransform(translation=(.5,-.1,.256),rotation=[0,1,0,0],from_frame='gripper',to_frame='world')
    reoriented = RigidTransform(translation=(.5,-.1,.256),rotation=[0.0018,-.85,-.464,-.249],from_frame='gripper',to_frame='world')
    # lpath, rpath = yk.compute_cartesian_path(l_start=L_HOME_POSE,l_goal=reoriented,l_qinit=YuMiKinematics.L_NICE_STATE)
    # print("l_waypoints=[")
    # for q in lpath:
    #     y=YuMiKinematics.urdf_order_2_yumi(np.rad2deg(q))
    #     print("[%f,%f,%f,%f,%f,%f,%f],"%(y[0],y[1],y[2],y[3],y[4],y[5],y[6]))
    # print("]")

    #right arm stuff
    lpath, rpath = yk.compute_cartesian_path(r_start=R_HOME_POSE,r_goal=reoriented,r_qinit=YuMiKinematics.R_NICE_STATE)
    print("r_waypoints=[")
    for q in rpath:
        y=YuMiKinematics.urdf_order_2_yumi(np.rad2deg(q))
        print("[%f,%f,%f,%f,%f,%f,%f],"%(y[0],y[1],y[2],y[3],y[4],y[5],y[6]))
    print("]")
    
