try:
    from yumiplanning.ompl_planner.yumiplanning_ompl import CollisionChecker,DualArmPlanner,SingleArmPlanner
except:
    print("\nCouldn't import ompl planner!\nMake sure you have built the yumi ompl planner in yumiplanning/ompl_planner\n")
'''
To use this class, make sure you build the planner in the ompl_planner folder. see README
'''
import numpy as np
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
            if len(path)==0:return None,None
            path=np.array(path)
            return path[:,:7],path[:,7:]
        if l_goal is not None:
            path=self.left.planPath(l_start,l_goal,r_start,timeout)
            if len(path)==0:return None
            return np.array(path)
        if r_goal is not None:
            path=self.right.planPath(r_start,r_goal,l_start,timeout)
            if len(path)==0:return None
            return np.array(path)

    def plan_to_pose(self,l_start_q, r_start_q, l_goal_p, r_goal_p):
        '''
        Returns path(s) from the start location to goal poses
        '''
        pass

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
    y=YuMi()
    planner=Planner()
    l_start=y.left.get_joints()
    r_start=y.right.get_joints()
    l_goal=YK.L_NICE_STATE
    r_goal=YK.R_NICE_STATE
    #plan just the left arm at a time
    # l_path=planner.plan(l_start,r_start,l_goal)
    # y.left.move_joints_traj(l_path,speed=(.1,2))
    # y.left.sync()
    # y.right.sync()
    #plan both and move in sync
    l_path,r_path=planner.plan(l_start,r_start,l_goal,r_goal,1)
    y.move_joints_sync(l_path,r_path,speed=(.1,2))
    y.left.sync()
    y.right.sync()