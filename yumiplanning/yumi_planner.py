try:
    from yumiplanning.ompl_planner.yumiplanning_ompl import CollisionChecker,DualArmPlanner,SingleArmPlanner
except:
    print("\nCouldn't import ompl planner!\nMake sure you have built the yumi ompl planner in yumiplanning/ompl_planner\n")
from autolab_core.rigid_transformations import RigidTransform
from yumiplanning.yumi_kinematics import YuMiKinematics
'''
To use this class, make sure you build the planner in the ompl_planner folder. see README
'''
import numpy as np
class PlanningException(Exception):
    def __init__(self, *args, **kwargs):
        Exception.__init__(self, *args, **kwargs)
class Planner:
    def __init__(self):
        import os
        desc_path = os.path.dirname(os.path.abspath(__file__)) + "/../yumi_description/"
        self.coll = CollisionChecker(desc_path)
        self.dual = DualArmPlanner(self.coll)
        self.left = SingleArmPlanner(self.coll,True,True)
        self.right = SingleArmPlanner(self.coll,False,True)

    def plan(self,l_start,r_start,l_goal=None,r_goal=None,timeout=1):
        '''
        plan a path from starts to goals. if only one goal is specified, planning will be done
        with single arm planner (other arm doesn't move). if two goals are specified, both arms
        will be planned for at once
        '''
        if l_goal is not None and r_goal is not None:
            #plan both
            s=np.zeros(14)
            s[:7]=l_start;s[7:]=r_start
            g=np.zeros(14)
            g[:7]=l_goal;g[7:]=r_goal
            path=self.dual.planPath(s,g,timeout)
            if len(path)==0:raise PlanningException("Couldn't plan path")
            path=np.array(path)
            return path[:,:7],path[:,7:]
        if l_goal is not None:
            path=self.left.planPath(l_start,l_goal,r_start,timeout)
            if len(path)==0:raise PlanningException("Couldn't plan path")
            return np.array(path)
        if r_goal is not None:
            path=self.right.planPath(r_start,r_goal,l_start,timeout)
            if len(path)==0:raise PlanningException("Couldn't plan path")
            return np.array(path)

    def plan_to_pose(self,l_start_q, r_start_q, yk, l_goal_p=None, r_goal_p=None):
        '''
        Returns path(s) from the start location to goal poses
        '''
        if not(isinstance(yk,YuMiKinematics)):
            raise Exception("plan_to_pose must be given a YuMiKinematics object")
        def isvalidiksol(lsol,rsol):
            if l_goal_p is None:lsol=l_start_q
            if r_goal_p is None:rsol=r_start_q
            if lsol is None or rsol is None:return False
            return self.is_valid_state(lsol,rsol)
        l_goal,r_goal = yk.ik(l_goal_p,r_goal_p,l_start_q,r_start_q,"Speed")
        if not isvalidiksol(l_goal,r_goal):
            for i in range(50):
                print("trying goal states")
                #try randomly a few times to see if we find one that doesn't collide
                l_goal,r_goal = yk.ik(l_goal_p,r_goal_p,None,None,"Speed")
                if isvalidiksol(l_goal,r_goal):
                    break
        if not isvalidiksol(l_goal,r_goal):raise PlanningException("Couldn't find valid goal states to reach goal poses")
        return self.plan(l_start_q,r_start_q,l_goal,r_goal)

    def is_valid_state(self,l_q,r_q):
        return not self.coll.isColliding(l_q,r_q)

if __name__=='__main__':
    # s=np.zeros(14)
    # g=np.zeros(14)
    # s[:7]=[1.7886884440180186, -0.44371156619454205, -1.7481374222057635, -0.40361105137018755, 0.16508589386459488, 1.1032037623419368, -0.1919595195294482]
    # g[:7]=[1.7889450395890478, 0.5727117937361313, -0.1202846167553847, 1.105436756336941, -0.8347407460212457, -0.9100656608720741, -0.17140514236767576]
    # planner=Planner()
    # l_path,r_path=planner.plan(s[:7],s[7:],g[:7],g[7:])
    # print(l_path,r_path)

    from yumirws.yumi import YuMi
    from yumiplanning.yumi_kinematics import YuMiKinematics as YK
    import time
    y=YuMi()
    yk=YuMiKinematics()
    ABB_WHITE = RigidTransform(translation=[0,0,.1325])
    METAL_GRIPPER = RigidTransform(translation=[0,0,.165])
    GRIP_DOWN_R = np.diag([1,-1,-1])#orientation where the gripper is facing downwards
    yk.set_tcp(METAL_GRIPPER.as_frames(yk.l_tcp_frame,yk.base_frame),ABB_WHITE.as_frames(yk.r_tcp_frame,yk.base_frame))
    y.left.close_gripper()
    y.right.close_gripper()
    planner=Planner()
    l_start=y.left.get_joints()
    r_start=y.right.get_joints()
    l_goal=YK.L_NICE_STATE
    r_goal=YK.R_NICE_STATE
    speed=.2
    #plan just the left arm at a time
    # l_path=planner.plan(l_start,r_start,l_goal)
    # y.left.move_joints_traj(l_path,speed=(.1,2))
    # y.left.sync()
    # y.right.sync()
    #plan both and move in sync
    l_path,r_path=planner.plan(l_start,r_start,l_goal,r_goal,1)
    y.move_joints_sync(l_path,r_path,speed=(speed,2))
    y.left.sync()
    y.right.sync()
    l_start=y.left.get_joints()
    r_start=y.right.get_joints()
    l=RigidTransform(translation=[.45,-.1,.1],rotation=GRIP_DOWN_R,from_frame=yk.l_tcp_frame,to_frame=yk.base_frame)
    r=RigidTransform(translation=[.55,0.1,.1],rotation=GRIP_DOWN_R,from_frame=yk.r_tcp_frame,to_frame=yk.base_frame)
    l_path,r_path=planner.plan_to_pose(l_start,r_start,yk,l_goal_p=l,r_goal_p=r)
    y.move_joints_sync(l_path,r_path,speed=(speed,2))
    y.left.sync()
    y.right.sync()
    l_start=y.left.get_joints()
    r_start=y.right.get_joints()
    l_path,r_path=planner.plan(l_start,r_start,l_goal,r_goal,1)
    y.move_joints_sync(l_path,r_path,speed=(speed,2))
    y.left.sync()
    y.right.sync()