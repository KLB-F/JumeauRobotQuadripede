# -*- coding: utf-8 -*-
"""
Code de la partie MGD/MGI et calcul de Trajectoire

Responsable : Le Bellec Killian
"""

import numpy as np
from Robot import Robot
import matplotlib.pyplot as plt
from Robot import Robot
from Trajectoire import Trajectoire
from Moteur import Moteur
from Patte import Patte

import meilleurTraj

l1, l2, l3 = [1.48, 0.90, 0.43] #En dm #Dimension du robot

robot = Robot(l1, l2, l3)

m1, m2, m3 = Moteur(0, -np.pi/2, np.pi/2, 0.7), Moteur(0, -np.pi/2, np.pi/2, 0.7), Moteur(0, -np.pi/2, np.pi/2, 0.7) #Moteur

patte = Patte(l1, l2, l3,[[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]], m1, m2, m3) #Création d'une patte


#1er exemple : Optimisation d'une trajectoire via algorithme génétique
""" 
traj = Trajectoire([np.array(patte.MGD(-np.pi/3, np.pi/2, 0)), np.array(patte.MGD(np.pi/4, np.pi/2, 0)), np.array(patte.MGD(np.pi/4, 0, 0)),np.array(patte.MGD(-np.pi/4, 0, 0)), np.array(patte.MGD(-np.pi/3, np.pi/2, 0))], [True, True, False, False, False], 0.1)
q0 = [[-np.pi/3, np.pi/2, 0], [np.pi/4, np.pi/2, 0], [np.pi/4, 0, 0],[-np.pi/4, 0, 0], [-np.pi/3, np.pi/2, 0]]
BP = [True, True, False, False, False]

qB = meilleurTraj.cherche_best_traj(robot, q0, BP, 1, 99)
print(qB)
traj = Trajectoire([np.array(patte.MGD(*qB[i])) for i in range(len(qB))], BP)
"""

#2eme exemple : importation d'une trajectoire utilisé pour la rotation du robot, transformation en une trajectoire pour marher droit puis intégration dans le robot
"""
traj = patte.load_Traj("600S")
trajbis = Trajectoire([np.array(patte.MGD(*[-patte.MGI(*[e[0], e[1], e[2]])[0], patte.MGI(*[e[0], e[1], e[2]])[1], patte.MGI(*[e[0], e[1], e[2]])[2]])) for e in traj.getTraj()], traj.getBP())

robot.robot_setTrajCustom(traj, trajbis, traj, trajbis)
"""

#3eme exemple création d'une trajectoire, exportation en gif, et exportation de celle-ci
"""
traj = Trajectoire([[np.float64(1.1288882941162778e-16), np.float64(-1.8436144934233345), np.float64(-1.2532766850655326)],
 [np.float64(1.3040192365800187),
 np.float64(-1.3041713156618442),
 np.float64(-1.2530301585780423)],
 [np.float64(1.844366316292662),
 np.float64(-0.00021508362312282694),
 np.float64(-1.2530301256885201)],
 [np.float64(1.3042547052222195), np.float64(1.3037984919994077), np.float64(-1.2530301695412167)], 
 [np.float64(2.2236661904130406), np.float64(0.32221111600885183), np.float64(-0.9010423215718447)], 
 [np.float64(2.043739049130081), np.float64(-1.2038416064281223), np.float64(-0.5501720648071368)], 
 [np.float64(1.1288882941162778e-16), np.float64(-1.8436144934233345), np.float64(-1.2532766850655326)]], [True, True, True, True, False, False, False])

robot.robot_setTraj(traj)
robot.aff_RobotAnim(0.2, 450)
#On reset
robot.robot_setTraj(traj)
robot.exportTrajBis("trajectoire_exporte")
"""

""" DEBUGAGE SEULEMENT

fig, axs = plt.subplots(5, sharex=True)

q0, q1, q2 = [], [], []
q3, q4, q5 = [], [], []
RPb0, RPb1 = [],[]
BP0, BP1 = [],[]
TN0, TN1 = [],[]
posS0, posS1 = [], []
T = []
for i in range(500):
    q = robot.PatteHG.getPosAngMot()
    q0.append(q[0])
    q1.append(q[1])
    q2.append(q[2])
    q = robot.PatteBG.getPosAngMot()
    q3.append(q[0])
    q4.append(q[1])
    q5.append(q[2])
    RPb0.append(robot.PatteHG.signal_getRP())
    RPb1.append(robot.PatteBG.signal_getRP())
    robot.robot_udpate(0.5)
    T.append(i*0.5)
    BP0.append(robot.PatteHG.Straj.getBP()[robot.PatteHG.posStraj[1]])
    BP1.append(robot.PatteBG.Straj.getBP()[robot.PatteBG.posStraj[1]])
    TN0.append(robot.PatteHG.signal_getTN())
    TN1.append(robot.PatteBG.signal_getTN())
    posS0.append(robot.PatteHG.posStraj[1])
    posS1.append(robot.PatteBG.posStraj[1])

axs[0].plot(T, q1, label="q1 - HG")
axs[0].plot(T, q2, label="q2 - HG")
axs[0].plot(T, q4, label="q1 - BG")
axs[0].plot(T, q5, label="q1 - BG")
axs[1].plot(T, RPb0, label="HG")
axs[1].plot(T, RPb1, label="BG")

axs[2].plot(T, BP0, label="HG")
axs[2].plot(T, BP1, label="BG")

axs[3].plot(T, TN0, label="HG")
axs[3].plot(T, TN1, label="BG")

axs[0].legend()
axs[1].legend()
axs[2].legend()
axs[3].legend()

axs[2].set_title("RP")
axs[1].set_title("RP poss.")
axs[3].set_title("TN")

axs[4].plot(T, posS0, label="HG")
axs[4].plot(T, posS1, label="BG")
axs[4].legend()
axs[4].set_title("pos. Traj")
axs[4].axhline(y=1.5, color='red')

plt.show()
"""
