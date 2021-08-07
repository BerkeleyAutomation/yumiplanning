try:
    from yumiplanning.ompl_planner.yumiplanning_ompl import CollisionChecker,DualArmPlanner,SingleArmPlanner
except:
    print("\nCouldn't import ompl planner!\nMake sure you have built the yumi ompl planner in yumiplanning/ompl_planner\n")
'''
To use this class, make sure you build
'''
import numpy as np
class Planner:
    def __init__(self):
        import os
        desc_path = os.path.dirname(os.path.abspath(__file__)) + "/../yumi_description/"
        self.c=CollisionChecker(desc_path)
        self.dual = DualArmPlanner(self.c)
        self.left = SingleArmPlanner(self.c,True,True)
        self.right = SingleArmPlanner(self.c,False,True)


if __name__=='__main__':
    s=np.zeros(14)
    g=np.zeros(14)
    s[:7]=[1.7886884440180186, -0.44371156619454205, -1.7481374222057635, -0.40361105137018755, 0.16508589386459488, 1.1032037623419368, -0.1919595195294482]
    g[:7]=[1.7889450395890478, 0.5727117937361313, -0.1202846167553847, 1.105436756336941, -0.8347407460212457, -0.9100656608720741, -0.17140514236767576]
    planner=Planner()
    path=planner.dual.planPath(s,g,3)
    print(np.array(path))