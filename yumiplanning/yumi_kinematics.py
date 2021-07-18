from tracikpy import TracIKSolver
from autolab_core import RigidTransform
import numpy as np

class YuMiKinematics():
    #these states are high manipulability configurations with elbows up and out,
    #and grippers facing perpendicularly down
    L_NICE_STATE=np.deg2rad(np.array([-71.52785377, -62.91241387, 61.09700753, 17.98294573,
             108.93368164,  75.65987325, 139.55185761]))
    R_NICE_STATE=np.array([ 1.21442839, -1.03205606, -1.10072738,  0.2987352,  
             -1.85257716,  1.25363652,-2.42181893])
    L_NICE_POSE = RigidTransform(translation=(.5,.2,.256),rotation=[0,1,0,0],from_frame='gripper_l_base',to_frame='base_link')
    R_NICE_POSE = RigidTransform(translation=(.5,-.2,.256),rotation=[0,1,0,0],from_frame='gripper_l_base',to_frame='base_link')
    #these are the names of frames in RigidTransforms
    base_frame="base_link"
    #tip frame is the end of the urdf file, in this case meaning the wrist
    l_tip_frame="gripper_l_base"
    r_tip_frame="gripper_r_base"
    #tcp is tool center point, meaning the point ik and fk will compute to
    l_tcp_frame="l_tcp"
    r_tcp_frame="r_tcp"
    
    def __init__(self):
        '''
        Initializes the kinematics with the urdf file
        '''
        import os
        urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/../yumi_description/urdf/yumi.urdf"
        
        #setup the tool center point as 0 transform
        self.l_tcp = RigidTransform(from_frame=self.l_tcp_frame,to_frame=self.l_tip_frame)
        self.r_tcp = RigidTransform(from_frame=self.r_tcp_frame,to_frame=self.r_tip_frame)
        '''taken from tracik:
        Speed: returns very quickly the first solution found
        % Distance: runs for the full timeout_in_secs, then returns the solution that minimizes SSE from the seed
        % Manipulation1: runs for full timeout, returns solution that maximizes sqrt(det(J*J^T)) (the product of the singular values of the Jacobian)
        % Manipulation2: runs for full timeout, returns solution that minimizes the ratio of min to max singular values of the Jacobian.
'''
        self.left_solvers={}
        self.left_solvers["Distance"]=TracIKSolver(urdf_path,self.base_frame,self.l_tip_frame,timeout=.05,solve_type="Distance")
        self.left_solvers["Manipulation1"] = TracIKSolver(urdf_path,self.base_frame,self.l_tip_frame,timeout=.05,solve_type="Manipulation1")
        self.left_solvers["Speed"] = TracIKSolver(urdf_path,self.base_frame,self.l_tip_frame,timeout=.05,solve_type="Speed")

        self.right_solvers={}
        self.right_solvers["Distance"]= TracIKSolver(urdf_path,self.base_frame,self.r_tip_frame,timeout=.05,solve_type="Distance")
        self.right_solvers["Manipulation1"] = TracIKSolver(urdf_path,self.base_frame,self.r_tip_frame,timeout=.05,solve_type="Manipulation1")
        self.right_solvers["Speed"] = TracIKSolver(urdf_path,self.base_frame,self.r_tip_frame,timeout=.05,solve_type="Speed")

    def set_tcp(self,l_tool=None,r_tool=None):
        if l_tool is None:
            self.l_tcp = RigidTransform(from_frame=self.l_tcp_frame,to_frame=self.l_tip_frame)
        else:
            assert(l_tool.from_frame == self.l_tcp_frame,l_tool.to_frame == self.l_tip_frame)
            self.l_tcp = l_tool
        if r_tool is None:
            self.r_tcp = RigidTransform(from_frame=self.r_tcp_frame,to_frame=self.r_tip_frame)
        else:
            assert(r_tool.from_frame == self.r_tcp_frame, r_tool.to_frame == self.r_tip_frame)
            self.r_tcp = r_tool

    def ik(self, left_pose=None, right_pose=None, left_qinit=None, right_qinit=None, solve_type="Speed"):
        '''
        given left and/or right target poses, calculates the joint angles and returns them as a tuple
        poses are RigidTransforms, qinits are np arrays
        solve_type can be "Distance" or "Manipulation1" (See constructor for what these mean)
        '''
        #NOTE: assumes given poses are in the l_tcp and r_tcp frames
        lres=None
        rres=None
        if left_pose is not None:
            left_pose = left_pose*self.l_tcp.inverse()
            lres=self.left_solvers[solve_type].ik(left_pose.matrix,left_qinit)
        if right_pose is not None:
            right_pose = right_pose*self.r_tcp.inverse()
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
            lpos = self.left_solvers["Distance"].fk(qleft)
            lres = RigidTransform(translation=lpos[:3,3],rotation=lpos[:3,:3],from_frame=self.l_tip_frame,to_frame=self.base_frame)*self.l_tcp
        if qright is not None:
            rpos = self.right_solvers["Distance"].fk(qright)
            rres = RigidTransform(translation=rpos[:3,3],rotation=rpos[:3,:3],from_frame=self.r_tip_frame,to_frame=self.base_frame)*self.r_tcp
        return (lres,rres)

    def compute_cartesian_path(self,l_start=None,l_goal=None,l_qinit=None,r_start=None,r_goal=None,r_qinit=None,
            N=10,jump_thresh=.3):
        '''
        Returns a traj interpolated linearly in cartesian space along the path between start and end
        all poses should be RigidTransform objects
        N is the mnumber of points to use when interpolating
        If no qinit is specified, the solver won't necessarily produce a close solution, so there may be
        a large motion to get to the first waypoint in the path
        '''
        #NOTE: assumes given poses are in the l_tcp and r_tcp frames
        lres=None
        rres=None
        if(l_start is not None and l_goal is not None):
           l_start = l_start*self.l_tcp.inverse()
           l_goal = l_goal*self.l_tcp.inverse()
           waypoints = l_start.linear_trajectory_to(l_goal,N)
           joint_traj = self.compute_traj(self.left_solvers,waypoints,jump_thresh,l_qinit)
           if self.check_jumps(joint_traj,jump_thresh):
               print("Warning: jump detected in left traj")
           lres = joint_traj
        if(r_start is not None and r_goal is not None):
            r_start = r_start*self.r_tcp.inverse()
            r_goal = r_goal*self.r_tcp.inverse()
            waypoints = r_start.linear_trajectory_to(r_goal,N)
            joint_traj = self.compute_traj(self.right_solvers,waypoints,jump_thresh,r_qinit)
            if self.check_jumps(joint_traj,jump_thresh):
               print("Warning: jump detected in right traj")
            rres=joint_traj
        return (lres,rres)

    def interpolate_cartesian_waypoints(self,l_waypoints=None,r_waypoints=None,
            l_qinit=None,r_qinit=None,N=20):
        '''
        Convenience function for chaining together a bunch of cartesian paths linearly
        makes successive calls to compute_cartesian_path
        '''
        #NOTE: assumes given poses are in the l_tcp and r_tcp frames
        lres=None
        rres=None
        if l_waypoints is not None:
            traj=[]
            for i in range(len(l_waypoints)-1):
                p,_=self.compute_cartesian_path(l_start=l_waypoints[i],l_goal=l_waypoints[i+1],
                        l_qinit=l_qinit,N=N)
                traj+=p
                l_qinit=traj[-1]
            lres=traj
        if r_waypoints is not None:
            traj=[]
            for i in range(len(r_waypoints)-1):
                _,p=self.compute_cartesian_path(r_start=r_waypoints[i],r_goal=r_waypoints[i+1],
                        r_qinit=r_qinit,N=N)
                traj+=p
                r_qinit=traj[-1]
            rres=traj
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
        #this call doesn't include the tcp because waypoints must already be in the gripper_{l,r}_base frame
        q_manip=solvers["Manipulation1"].ik(waypoints[0].matrix, qinit)
        if qinit is None or np.max(np.abs(q_manip-qinit))>jump_thresh:
            joint_traj.append(solvers["Speed"].ik(waypoints[0].matrix, qinit))
        else:
            joint_traj.append(q_manip)
        #then do the rest of the traj using faster solver
        for i in range(1,len(waypoints)):
            m = waypoints[i].matrix
            joint_traj.append(solvers["Speed"].ik(m,joint_traj[-1]))
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
    ltarget=RigidTransform(translation=[.5,.2,.1],rotation=[0,1,0,0],from_frame="l_tcp")
    rtarget=RigidTransform(translation=[.5,-.2,.1],rotation=[0,1,0,0],from_frame="r_tcp")
    lq,rq = yk.ik(ltarget,rtarget,left_qinit=YuMiKinematics.L_NICE_STATE,right_qinit=YuMiKinematics.R_NICE_STATE)
    print("joints without tcp:",lq)
    yk.set_tcp(RigidTransform(translation=[0,0,.156],from_frame="l_tcp",to_frame=yk.l_tip_frame),RigidTransform(translation=[0,0,.156],from_frame="r_tcp",to_frame=yk.r_tip_frame))
    lq,rq = yk.ik(ltarget,rtarget,left_qinit=YuMiKinematics.L_NICE_STATE,right_qinit=YuMiKinematics.R_NICE_STATE)
    lp,rp = yk.fk(lq,rq)
    print("joints with tcp:",lq)
    print(lp.translation,lp.quaternion)
