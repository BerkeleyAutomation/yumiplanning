from tracikpy import TracIKSolver
from autolab_core import RigidTransform
import numpy as np

def quat_dist(q1,q2):
    ret=np.arccos(min(1,max(-1,2*np.power(q1.dot(q2),2.0)-1)))
    return ret
def pick(tup):
    '''convenience for picking the non-None value in a 2-tuple
    '''
    return tup[1-tup.index(None)]
class YuMiKinematics():
    #these are the names of frames in RigidTransforms
    base_frame="base_link"
    #tip frame is the end of the urdf file, in this case meaning the wrist
    l_tip_frame="gripper_l_base"
    r_tip_frame="gripper_r_base"
    #tcp is tool center point, meaning the point ik and fk will compute to
    l_tcp_frame="l_tcp"
    r_tcp_frame="r_tcp"
    #these states are high manipulability configurations with elbows up and out,
    #and grippers facing perpendicularly down
    L_NICE_STATE=np.deg2rad(np.array([-71.52785377, -62.91241387, 61.09700753, 17.98294573,
             108.93368164,  75.65987325, 139.55185761]))
    R_NICE_STATE=np.array([ 1.21442839, -1.03205606, -1.10072738,  0.2987352,  
             -1.85257716,  1.25363652,-2.42181893])
    #these poses for the wrists (not tcp)
    L_NICE_POSE = RigidTransform(translation=(.5,.2,.256),rotation=[0,1,0,0],
        from_frame=l_tip_frame,to_frame=base_frame)
    R_NICE_POSE = RigidTransform(translation=(.5,-.2,.256),rotation=[0,1,0,0],
        from_frame=r_tip_frame,to_frame=base_frame)
    
    
    def __init__(self):
        '''
        Initializes the kinematics with the urdf file
        '''
        import os
        urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/../yumi_description/urdf/yumi.urdf"
        
        #setup the tool center point as 0 transform
        self.set_tcp(None,None)
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
        self.solvers={"left":self.left_solvers,"right":self.right_solvers}

    def set_tcp(self,l_tool=None,r_tool=None):
        if l_tool is None:
            self.l_tcp = RigidTransform(from_frame=self.l_tcp_frame,to_frame=self.l_tip_frame)
        else:
            assert l_tool.from_frame == self.l_tcp_frame,l_tool.to_frame == self.l_tip_frame
            self.l_tcp = l_tool
        if r_tool is None:
            self.r_tcp = RigidTransform(from_frame=self.r_tcp_frame,to_frame=self.r_tip_frame)
        else:
            assert r_tool.from_frame == self.r_tcp_frame, r_tool.to_frame == self.r_tip_frame
            self.r_tcp = r_tool

    def ik(self, left_pose=None, right_pose=None, left_qinit=None, right_qinit=None, solve_type="Speed",
            bs=[1e-5,1e-5,1e-5,  1e-3,1e-3,1e-3]):
        '''
        given left and/or right target poses, calculates the joint angles and returns them as a tuple
        poses are RigidTransforms, qinits are np arrays
        solve_type can be "Distance" or "Manipulation1" or "Speed" (See constructor for what these mean)
        bs is an array representing the tolerance on end pose of the gripper
        '''
        #NOTE: assumes given poses are in the l_tcp and r_tcp frames
        lres=None
        rres=None
        if left_pose is not None:
            left_pose = left_pose*self.l_tcp.inverse()
            lres=self.left_solvers[solve_type].ik(left_pose.matrix,left_qinit,
                    bx=bs[0],by=bs[1],bz=bs[2],  brx=bs[3],bry=bs[4],brz=bs[5])
        if right_pose is not None:
            right_pose = right_pose*self.r_tcp.inverse()
            rres=self.right_solvers[solve_type].ik(right_pose.matrix,right_qinit,
                    bx=bs[0],by=bs[1],bz=bs[2],  brx=bs[3],bry=bs[4],brz=bs[5])
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
            lpos = self.left_solvers["Speed"].fk(qleft)
            lres = RigidTransform(translation=lpos[:3,3],rotation=lpos[:3,:3],from_frame=self.l_tip_frame,to_frame=self.base_frame)*self.l_tcp
        if qright is not None:
            rpos = self.right_solvers["Speed"].fk(qright)
            rres = RigidTransform(translation=rpos[:3,3],rotation=rpos[:3,:3],from_frame=self.r_tip_frame,to_frame=self.base_frame)*self.r_tcp
        return (lres,rres)

    def refine_state(self,l_state=None,r_state=None,t_tol=.05,r_tol=.5):
        '''
        attempts to find a configuration that the robot can move to without
        deviating from its current position more than t_tol in translation and r_tol in orientation
        '''
        #technique 1: try linearly interpolating in joint space to desired config, and if the pose doesn't leave the bounds, return it.
        l_pos,r_pos = self.fk(l_state,r_state)
        l_des,r_des = self.ik(l_pos,r_pos,l_state,r_state,"Manipulation1")
        lres=[]
        rres=[]
        if l_state is not None:
            traj = np.linspace(l_state,l_des,20)
            tdev,rdev = self.max_deviation(traj,l_pos,"qleft")
            if(tdev<t_tol and rdev<r_tol):
                lres = traj
        if r_state is not None:
            traj = np.linspace(r_state,r_des,20)
            tdev,rdev = self.max_deviation(traj,r_pos,"qright")
            if(tdev<t_tol and rdev<r_tol):
                rres = traj
        return lres,rres

    def max_deviation(self,traj,pose,arm_name):
        '''
        finds the maximum deviation (meters,radians) of the tcp from the given pose
        arm_name is "qleft" or "qright"
        '''
        tmax=0
        rmax=0
        for q in traj:
            r = self.fk(**{arm_name:q})
            p = pick(r)
            tmax = max(np.linalg.norm(p.translation-pose.translation),tmax)
            rmax = max(quat_dist(p.quaternion,pose.quaternion),rmax)
        return tmax,rmax

    def compute_cartesian_path(self,l_start=None,l_goal=None,l_qinit=None,r_start=None,r_goal=None,r_qinit=None,
            N=10,jump_thresh=.4):
        '''
        Returns a traj interpolated linearly in cartesian space along the path between start and end
        all poses should be RigidTransform objects
        N is the mnumber of points to use when interpolating
        If no qinit is specified, the solver won't necessarily produce a close solution, so there may be
        a large motion to get to the first waypoint in the path
        
        mid_tol and end_tol give tolerances on deviation from middle waypoints and final waypoint
        the first coordinate is translational deviation, and second is rotational deviation
        '''
        #NOTE: assumes given poses are in the l_tcp and r_tcp frames
        lres=[]
        rres=[]
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
        lres=[]
        rres=[]
        if l_waypoints is not None and len(l_waypoints)>1:
            traj=[]
            for i in range(len(l_waypoints)-1):
                p,_=self.compute_cartesian_path(l_start=l_waypoints[i],l_goal=l_waypoints[i+1],
                        l_qinit=l_qinit,N=N)
                traj+=p
                l_qinit=traj[-1]
            lres=traj
        if r_waypoints is not None and len(r_waypoints)>1:
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
        for i in range(1,len(waypoints)-1):
            m = waypoints[i].matrix
            joint_traj.append(solvers["Speed"].ik(m,joint_traj[-1],brx=.1,bry=.1,brz=.1))
        m = waypoints[-1].matrix
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
            diffvec=abs(joint_traj[i]-tmp)
            diff=np.max(diffvec)
            if diff>jump_thresh:
                print("Jump of magnitude",diff,"found")
                if(np.max(diffvec[0:4])>jump_thresh):
                    raise Exception("Jump in first 4 joints, aborting")
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
    @staticmethod
    def yumi_format_2_urdf(q):
        return np.deg2rad(YuMiKinematics.yumi_order_2_urdf(q))

    @staticmethod
    def urdf_format_2_yumi(q):
        return np.rad2deg(YuMiKinematics.urdf_order_2_yumi(q))

if __name__=="__main__":
    pass
'''

<JointInfos>
<Element type="TJointInfo">
<IsRevoluteJoint Value="true"/>
<DHParameters>
<Element type="TDHParameters">
<NextJoint Value="1"/>
<Link Value="1"/>
<Twist Value="0"/>
<Length Value="0"/>
<Rotation Value="0"/>
<Offset Value="0"/>
</Element>
</DHParameters>
</Element>
<Element type="TJointInfo">
<IsRevoluteJoint Value="true"/>
<DHParameters>
<Element type="TDHParameters">
<NextJoint Value="2"/>
<Link Value="2"/>
<Twist Value="1.57079632679558"/>
<Length Value="0.0300000000000276"/>
<Rotation Value="3.14159265358979"/>
<Offset Value="1.51574274386155E-13"/>
</Element>
</DHParameters>
</Element>
<Element type="TJointInfo">
<IsRevoluteJoint Value="true"/>
<DHParameters>
<Element type="TDHParameters">
<NextJoint Value="3"/>
<Link Value="3"/>
<Twist Value="1.57079632679569"/>
<Length Value="0.0299999999999672"/>
<Rotation Value="0"/>
<Offset Value="0.251499999999955"/>
</Element>
</DHParameters>
</Element>
<Element type="TJointInfo">
<IsRevoluteJoint Value="true"/>
<DHParameters>
<Element type="TDHParameters">
<NextJoint Value="4"/>
<Link Value="4"/>
<Twist Value="-1.57079632679535"/>
<Length Value="0.0404999999999989"/>
<Rotation Value="-1.57079632679452"/>
<Offset Value="-8.95329360786013E-14"/>
</Element>
</DHParameters>
</Element>
<Element type="TJointInfo">
<IsRevoluteJoint Value="true"/>
<DHParameters>
<Element type="TDHParameters">
<NextJoint Value="5"/>
<Link Value="5"/>
<Twist Value="-1.57079632679503"/>
<Length Value="0.0405000000001106"/>
<Rotation Value="3.14159265358979"/>
<Offset Value="0.264999999999955"/>
</Element>
</DHParameters>
</Element>
<Element type="TJointInfo">
<IsRevoluteJoint Value="true"/>
<DHParameters>
<Element type="TDHParameters">
<NextJoint Value="6"/>
<Link Value="6"/>
<Twist Value="-1.57079632679442"/>
<Length Value="0.0269999999999218"/>
<Rotation Value="3.14159265358979"/>
<Offset Value="-4.95470001989013E-14"/>
</Element>
</DHParameters>
</Element>
<Element type="TJointInfo">
<IsRevoluteJoint Value="true"/>
<DHParameters>
<Element type="TDHParameters">
<NextJoint Value="-1"/>
<Link Value="7"/>
<Twist Value="-1.5707963267943"/>
<Length Value="0.0269999999998913"/>
<Rotation Value="0"/>
<Offset Value="1.90227280103862E-17"/>
</Element>
</DHParameters>
</Element>
</JointInfos>
'''
