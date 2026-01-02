"""
Algorithme déterminant la meilleur trajectoire à suivre
"""

from Robot import Robot
import matplotlib.pyplot as plt
from Robot import Robot
from Trajectoire import Trajectoire
from Moteur import Moteur
from Patte import Patte
import numpy as np
import numpy.random as rnd
import copy

from tqdm import trange #On veut une belle bar de progression



def cherche_best_traj(robot:Robot, q0:[[float, float]], BP:[bool], max_iter:int = 100, nb_agent:int = 60,seed:int=None) -> [[float, float]]:
    """
    Recherche la meilleur trajectoire par algoirthme génétique
    
    Rq : nb_agent doit être divisible par 3
    """
    #BP fixe
    plt.clf()
    sc_l = []
    
    #Fonction de mutation
    def mutation_traj(qTraj, eps):
        nonlocal robot
        i = rnd.randint(0, len(qTraj)-1)
        j = rnd.randint(0, 2)
        qTraj[i][j] += (rnd.random()-1/2)*eps
        if j == 0:
            if robot.PatteHD.M1.get_Param()[0] > qTraj[i][j]:
                qTraj[i][j] = robot.PatteHD.M1.get_Param()[0]
            elif robot.PatteHD.M1.get_Param()[1] < qTraj[i][j]:
                qTraj[i][j] = robot.PatteHD.M1.get_Param()[1]
        elif j == 1:
            if robot.PatteHD.M2.get_Param()[0] > qTraj[i][j]:
                qTraj[i][j] = robot.PatteHD.M2.get_Param()[0]
            elif robot.PatteHD.M2.get_Param()[1] < qTraj[i][j]:
                qTraj[i][j] = robot.PatteHD.M2.get_Param()[1]
        elif j == 2:
            if robot.PatteHD.M3.get_Param()[0] > qTraj[i][j]:
                qTraj[i][j] = robot.PatteHD.M2.get_Param()[0]
            elif robot.PatteHD.M2.get_Param()[1] < qTraj[i][j]:
                qTraj[i][j] = robot.PatteHD.M3.get_Param()[1]
        qTraj[i][2] = max(min(np.pi/2-qTraj[i][1], np.pi/2), -np.pi/2) #Condition d'orthogonalité
        if i == 0:
            qTraj[-1] = qTraj[0][:]
    
    def f_eps(iteration:int):
        nonlocal max_iter
        return 0.07*np.exp(-iteration/(max_iter/3)) 
    
    if seed != None:
        rnd.seed(seed)
    
    qTraj = [copy.deepcopy(q0) for i in range(nb_agent)]
    
    
    itera = 0
    
    for itera in trange(max_iter, desc="Générations "):
        #Mutation
        eps = f_eps(itera)
        for i in range(nb_agent):
            mutation_traj(qTraj[i], eps)
        
        #Selection
        score = []
        
        for i in range(nb_agent):
            robot.robot_setTraj(Trajectoire([robot.PatteHD.MGD(*qTraj[i][j]) for j in range(len(qTraj[i]))], BP))
            score.append([robot.robot_evalTrajSprint(), i])
        
        b = sorted(score, reverse=True)[:int(nb_agent/3)]
        
        sc_l.append((b[0][0]+b[1][0]+b[2][0]+b[3][0]+b[4][0])/5) #Moyenne des 5 meilleurs algo.
        #Reproduction
        qTrajN = []
        for s, i in b:
            qTrajN.append(copy.deepcopy(qTraj[i]))
            mutation_traj(copy.deepcopy(qTraj[i]), eps)
            qTrajN.append(copy.deepcopy(qTraj[i]))
            mutation_traj(copy.deepcopy(qTraj[i]), eps)
            qTrajN.append(copy.deepcopy(qTraj[i]))
        
        qTraj = copy.deepcopy(qTrajN)        
        
        itera+= 1
    
    plt.plot([i for i in range(len(sc_l))], sc_l)
    plt.title("Evolution du score des 5 meilleurs trajectoires en fonction des générations")
    plt.ylabel("Score")
    plt.xlabel("Générations")
    plt.show()
    
    return qTraj[b[0][1]]