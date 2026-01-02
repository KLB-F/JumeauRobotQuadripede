"""
Classe Robot, qui s'occupe de simuler le robot
"""
from Objet import Objet
from Patte import Patte
from Moteur import Moteur
from Trajectoire import Trajectoire
import numpy as np
from numpy import cos, sin
from PIL import Image
import csv

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D


class Robot(Objet):
    def __init__(self, l1:float, l2:float, l3:float, mMoteur:float = 0.055, mCorps:float = 0.715):
        """
        Paramètres :
            l1, l2, l3 : les longueurs caractéristiques des pattes du robot
            mMoteur, mCorps la masse d'un moteur et du corps
        """
        self.mMoteur = mMoteur
        self.mCorps = mCorps
        self.l1, self.l2, self.l3 = l1, l2, l3
        
        alpha = np.pi/4 #Angle entre le repère du robot et le repère de la patte selon z
        self.d1, self.d2, self.d3, self.e1, self.e2 = 1*0.75, 1*0.75, 1*0.75, 0.5*0.75, 0.5*0.75 #Carcéristique de la pièce centrale du robot
        self.r1, self.r2, self.r3 = 0.75, 0.75, 0 #Distance entre le repère est la patte haute droite par apport au centre de gravité G
        
        #Patte Haute Droite
        TpatteHD = [[np.cos(alpha), -np.sin(alpha), 0, self.r1], [np.sin(alpha), np.cos(alpha), 0, self.r2], [0, 0, 1, self.r3], [0, 0, 0, 1]]
        self.mHD1, self.mHD2, self.mHD3 = Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1)
        
        self.PatteHD = Patte(l1, l2, l3, TpatteHD, self.mHD1, self.mHD2, self.mHD3)
        
        #Patte Haute Gauche
        TpatteHG = [[np.cos(-alpha), -np.sin(-alpha), 0, self.r1], [np.sin(-alpha), np.cos(-alpha), 0, -self.r2], [0, 0, 1, self.r3], [0, 0, 0, 1]]
        self.mHG1, self.mHG2, self.mHG3 = Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1)
        
        self.PatteHG = Patte(l1, l2, l3, TpatteHG, self.mHG1, self.mHG2, self.mHG3)
        
        #Patte Basse Droite
        TpatteBD = [[np.cos(np.pi-alpha), -np.sin(np.pi-alpha), 0, -self.r1], [np.sin(np.pi-alpha), np.cos(np.pi-alpha), 0, self.r2], [0, 0, 1, self.r3], [0, 0, 0, 1]]
        self.mBD1, self.mBD2, self.mBD3 = Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1)
        
        self.PatteBD = Patte(l1, l2, l3, TpatteBD, self.mBD1, self.mBD2, self.mBD3)
        
        #Patte Basse Gauche
        TpatteBG = [[np.cos(np.pi+alpha), -np.sin(np.pi+alpha), 0, -self.r1], [np.sin(np.pi+alpha), np.cos(np.pi+alpha), 0, -self.r2], [0, 0, 1, self.r3], [0, 0, 0, 1]]
        self.mBG1, self.mBG2, self.mBG3 = Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1), Moteur(0, -np.pi/2, np.pi/2, 0.1)
        
        self.PatteBG = Patte(l1, l2, l3, TpatteBG, self.mBG1, self.mBG2, self.mBG3)
    
    def getPos():
        return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0 , 1]]
    
    def __aff_RobotInt(self, q:[float]):
        """
        Fonction intermediaire commune à aff_PosRobot & aff_RobotAnim
        
        Paramètre :
            q = [qHD, qHG, qBD, qBG]
        Où : qXY désigne les coordonnés articulaite de la patte XY
            H -> Haut; B -> Bas
            D -> Droite; G -> Gauche
            
        
        """
        qHD1, qHD2, qHD3, qHG1, qHG2, qHG3, qBD1, qBD2, qBD3, qBG1, qBG2, qBG3 = q
        x = [self.d1-self.e1, self.d1, self.d1, self.d1-self.e1, -self.d1+self.e1, -self.d1, -self.d1, self.e1-self.d1, self.d1-self.e1]
        y = [self.d2, self.d2-self.e2, -self.d2+self.e2, -self.d2, -self.d2, -self.d2+self.e2, self.d2-self.e2, self.d2, self.d2]
        z = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        #Patte Droite Haute
        X_trace = self.PatteHD.X_trace(qHD1, qHD2, qHD3)
        for i in range(len(X_trace)):
            #print([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1])
            X_trace[i] = ((self.PatteHD.getPos()*np.transpose(np.matrix([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1]))))[:-1]
        
        xsHD = [float(p[0]) for p in X_trace]
        ysHD = [float(p[1]) for p in X_trace]
        zsHD = [float(p[2]) for p in X_trace]
        
        #Patte Gauche Haute
        X_trace = self.PatteHG.X_trace(qHG1, qHG2, qHG3)
        for i in range(len(X_trace)):
            #print([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1])
            X_trace[i] = ((self.PatteHG.getPos()*np.transpose(np.matrix([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1]))))[:-1]
        
        xsHG = [float(p[0]) for p in X_trace]
        ysHG = [float(p[1]) for p in X_trace]
        zsHG = [float(p[2]) for p in X_trace]
        
        #Patte Droite Basse
        X_trace = self.PatteBD.X_trace(qBD1, qBD2, qBD3)
        for i in range(len(X_trace)):
            #print([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1])
            X_trace[i] = ((self.PatteBD.getPos()*np.transpose(np.matrix([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1]))))[:-1]
        
        xsBD = [float(p[0]) for p in X_trace]
        ysBD = [float(p[1]) for p in X_trace]
        zsBD = [float(p[2]) for p in X_trace]
        
        #Patte Gauche Basse
        X_trace = self.PatteBG.X_trace(qBG1, qBG2, qBG3)
        for i in range(len(X_trace)):
            #print([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1])
            X_trace[i] = ((self.PatteBG.getPos()*np.transpose(np.matrix([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1]))))[:-1]
        
        xsBG = [float(p[0]) for p in X_trace]
        ysBG = [float(p[1]) for p in X_trace]
        zsBG = [float(p[2]) for p in X_trace]
        
        return [xsHD, ysHD, zsHD, xsHG, ysHG, zsHG, xsBD, ysBD, zsBD, xsBG, ysBG, zsBG, x, y, z]
    
    def aff_PosRobot(self, q:[float]):
        """
        Affiche le robot en fonction des coordonnés articulaires q = [qHD, qHG, qBD, qBG] 
        Où : qXY désigne les coordonnés articulaite de la patte XY
            H -> Haut; B -> Bas
            D -> Droite; G -> Gauche
        """
        fig = plt.figure()
        ax = Axes3D(fig, auto_add_to_figure=False)
        ax_echelle = 5
        ax.axes.set_xlim3d(left=-ax_echelle, right=ax_echelle)
        ax.axes.set_ylim3d(bottom=-ax_echelle, top=ax_echelle)
        ax.axes.set_zlim3d(bottom=-2, top=1)
        fig.add_axes(ax)
        
        ax.legend()
        
        xsHD, ysHD, zsHD, xsHG, ysHG, zsHG, xsBD, ysBD, zsBD, xsBG, ysBG, zsBG, x, y, z = self.__aff_RobotInt(q)
        
        ax.plot(xsHD, ysHD, zsHD, '-k', linewidth=3, markersize=6)
        ax.plot(xsHD, ysHD, zsHD, 'or', linewidth=3, markersize=6)
        
        ax.plot(xsHG, ysHG, zsHG, '-k', linewidth=3, markersize=6)
        ax.plot(xsHG, ysHG, zsHG, 'or', linewidth=3, markersize=6)
        
        ax.plot(xsBD, ysBD, zsBD, '-k', linewidth=3, markersize=6)
        ax.plot(xsBD, ysBD, zsBD, 'or', linewidth=3, markersize=6)
        
        ax.plot(xsBG, ysBG, zsBG, '-k', linewidth=3, markersize=6)
        ax.plot(xsBG, ysBG, zsBG, 'or', linewidth=3, markersize=6)
        
        corps = [list(zip(x, y, z))]
        ax.add_collection(Poly3DCollection(corps, alpha=0.5))
        ax.scatter([0, 0], [0, 0], [0, 0.01], color="red")
        ax.set_xlabel("x (dm)")
        ax.set_ylabel("y (dm)")
        ax.set_zlabel("z (dm)")
        plt.show()
        
    def aff_RobotAnim(self, inter_t:float, nb_points:int) -> None:
        """
        Affiche une animation de la trajectoire sur un intervalle donné
        
        Paramètre : 
            inter_t : l'intervalle d'affichage
            nb_points : le nombre de point voulue
        """
            
        for i in range(nb_points):
            fig = plt.figure()
            ax = Axes3D(fig, auto_add_to_figure=False)
            ax_echelle = 5
            ax.axes.set_xlim3d(left=-ax_echelle, right=ax_echelle)
            ax.axes.set_ylim3d(bottom=-ax_echelle, top=ax_echelle)
            ax.axes.set_zlim3d(bottom=-ax_echelle, top=ax_echelle)
            fig.add_axes(ax)
            
            ax.legend()
            
            q = list(self.PatteHD.getPosAngMot())+list(self.PatteHG.getPosAngMot())+list(self.PatteBD.getPosAngMot())+list(self.PatteBG.getPosAngMot())
            xsHD, ysHD, zsHD, xsHG, ysHG, zsHG, xsBD, ysBD, zsBD, xsBG, ysBG, zsBG, x, y, z = self.__aff_RobotInt(q)
            self.robot_udpate(inter_t)
            
            ax.plot(xsHD, ysHD, zsHD, '-k', linewidth=3, markersize=6)
            ax.plot(xsHD, ysHD, zsHD, 'or', linewidth=3, markersize=6)
            
            ax.plot(xsHG, ysHG, zsHG, '-k', linewidth=3, markersize=6)
            ax.plot(xsHG, ysHG, zsHG, 'or', linewidth=3, markersize=6)
            
            ax.plot(xsBD, ysBD, zsBD, '-k', linewidth=3, markersize=6)
            ax.plot(xsBD, ysBD, zsBD, 'or', linewidth=3, markersize=6)
            
            ax.plot(xsBG, ysBG, zsBG, '-k', linewidth=3, markersize=6)
            ax.plot(xsBG, ysBG, zsBG, 'or', linewidth=3, markersize=6)
            
            corps = [list(zip(x, y, z))]
            ax.add_collection(Poly3DCollection(corps, alpha=0.5))
            ax.scatter([0, 0], [0, 0], [0, 0.01], color="red")
            plt.savefig("temp/"+str(i)+".png")
            plt.close()
        images = [Image.open("temp/"+str(i)+".png") for i in range(nb_points)]
        images[0].save("traj.gif", save_all=True, append_images=images[1:], duration=inter_t*nb_points/3, loop=1)

        
    def robot_udpate(self, dt:float) -> None:
        """
        Fonction d'udpate du robot
        
        Paramètre : 
            dt : l'intervalle de temps entre chaque update
        """
            
        self.PatteHD.patte_udpate(dt)
        self.PatteHG.patte_udpate(dt)
        self.PatteBG.patte_udpate(dt)
        self.PatteBD.patte_udpate(dt)
        
        self.PatteBD.signal_setRP(min(self.PatteHD.signal_getTN(), self.PatteBG.signal_getTN()))
        self.PatteHG.signal_setRP(min(self.PatteHD.signal_getTN(), self.PatteBG.signal_getTN()))
        
        self.PatteHD.signal_setRP(min(self.PatteBD.signal_getTN(), self.PatteHG.signal_getTN()))
        self.PatteBG.signal_setRP(min(self.PatteBD.signal_getTN(), self.PatteHG.signal_getTN()))
    
    def robot_setTraj(self, Traj:Trajectoire) -> None:
        """
        Set la trajectoire du robot à Traj
        """
        self.PatteHD.traj_Suivie(Traj, decalage=True)
        self.PatteHG.traj_Suivie(Traj)
        self.PatteBG.traj_Suivie(Traj, decalage=True)
        self.PatteBD.traj_Suivie(Traj)
    
    def __X_traceCG(self, q1, q2, q3):
        """
        Retourne la valeur de la matrice X_trace en position actuelle sauf pour l'effecteur
        A le mérite d'être un peu plus rapide que la version X_trace de base
        
        Paramètres : 
            q1, q2, q3 : les coordonnés articulaires
        
        Retourne : 
            X_trace
        """
        cosq1, sinq1, cosq2 = cos(q1), sin(q1), cos(q2)
        return [np.matrix([ [0], [0], [0], [1]]), 
                np.matrix([ [self.l1*cosq1], [self.l1*sinq1], [0], [1]]), 
                np.matrix([ [self.l1*cosq1 + self.l2*cosq1*cosq2], [self.l1*cosq1 + self.l2*sinq1*cosq2], [-self.l2*np.sin(q2)], [1]])]
    
    def robot_calculCG(self) -> [float, float, float]:
        """ Renvoie la position du centre de gravité dans la position actuelle """
        X_nom, Y_nom, Z_nom = 0, 0, 0 #Nomitateur de la moyenne sur X, Y, Z
        P = 0
        S_coef = self.mCorps #Dénominateur
        
        #Calcul du barycentre
        X_trace = self.__X_traceCG(*self.PatteBD.getPosAngMot())
        for i in range(len(X_trace)): #On transpose dans le repère du Robot & on enleve l'effecteur
            #print([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1])
            P = ((self.PatteBD.getPos()*X_trace[i]))[:-1]*self.mMoteur
            S_coef += self.mMoteur
        
        X_trace = self.__X_traceCG(*self.PatteBG.getPosAngMot())
        for i in range(len(X_trace)): #On transpose dans le repère du Robot & on enleve l'effecteur
            #print([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1])
            P = ((self.PatteBG.getPos()*X_trace[i]))[:-1]*self.mMoteur + P
            S_coef += self.mMoteur
        
        X_trace = self.__X_traceCG(*self.PatteHD.getPosAngMot())
        for i in range(len(X_trace)): #On transpose dans le repère du Robot & on enleve l'effecteur
            #print([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1])
            P = ((self.PatteHD.getPos()*X_trace[i]))[:-1]*self.mMoteur + P
            S_coef += self.mMoteur
        
        X_trace = self.__X_traceCG(*self.PatteHG.getPosAngMot())
        for i in range(len(X_trace)): #On transpose dans le repère du Robot & on enleve l'effecteur
            #print([float(X_trace[i][j]) for j in range(len(X_trace[i]))]+[1])
            P = ((self.PatteHG.getPos()*X_trace[i]))[:-1]*self.mMoteur + P
            S_coef += self.mMoteur
        X_nom, Y_nom, Z_nom = P
        return np.array(X_nom)[0][0]/S_coef, np.array(Y_nom)[0][0]/S_coef, np.array(Z_nom)[0][0]/S_coef
    
    def robot_calculStab(self) -> float:
        """
        Renvoie l'indice de stabilité, la distance entre le CG est la droite traversant les 2 pattes au sol
        """
        m, c = 0, 0
        xC, yC = self.robot_calculCG()[:-1]
        if self.PatteHD.signal_getRP() and self.PatteBG.signal_getRP():
            xA, yA = np.array(self.PatteHD.getPosEffecteur()[:-1])
            xB, yB = np.array(self.PatteBG.getPosEffecteur()[:-1])
            m = (yB-yA)/(xB-xA)
            c = yA-m*xA
        else:
            xA, yA = np.array(self.PatteHG.getPosEffecteur()[:-1])
            xB, yB = np.array(self.PatteBD.getPosEffecteur()[:-1])
            m = (yB-yA)/(xB-xA)
            c = yA-m*xA
        return float((abs(yC-m*xC-c)/np.sqrt(1+m**2))[0])
        
    
    def robot_evalTrajSprint(self) -> float:
        """
        
        Evalue la trajectoire actuelle

        Retourne:
            Le score de la trajectoire
        """
        
        max_iter = 600
        dt = 0.15
        itera = 0
        #traj_valide = True #Trajectoire valide ?
        
        score = 0
        
        C_check_1, C_check_2, P_check_0, P_check_1 = False, False, False, False #Pour savoir si l'on a fait un cycle de marche complet, on regarde si la patte en decalage est passe par sa position initial et si l'autre patte (pas décalé) à finit son TN
        
        #1er analyse : distance parcourue théorique
        traj = self.PatteBD.Straj.getTraj()
        BP_l = self.PatteBD.Straj.getBP()
        #Hyp : BP_l == True
        d_x, d_y, d_z = 0, 0, 0
        posP = np.array(self.PatteBD.Tbase*np.matrix(list(traj[0])+[1]).transpose())[:-1]
        for i in range(1,len(BP_l)):
            if BP_l[i] == True:
                posS = np.array(self.PatteBD.Tbase*np.matrix(list(traj[i])+[1]).transpose())[:-1]
                d_x += posS[0]-posP[0]
                d_y += posS[1]-posP[1]
                d_z += posS[2]-posP[2]
                posP = posS[:]
            else:
                break
        d_x, d_y, d_z = d_x[0], d_y[0], d_z[0]
                
        #On effectue une fois toute la trajectoire
        while (not (P_check_0 and P_check_1)) and max_iter > itera:
            self.robot_udpate(dt)
            
            #Zone de travail
            
            posHD = self.PatteHD.getPosEffecteur()
            posBD = self.PatteBD.getPosEffecteur()
            
            #Trajectoire valide ?
            if (self.PatteHD.posStraj[0] == False and self.PatteBD.posStraj[0] == True and posHD[2] < posBD[2]) or (self.PatteHD.posStraj[0] == True and self.PatteBD.posStraj[0] == False and posHD[2] > posBD[2]):
                #traj_valide = False
                return -np.inf #Trajectoire non valide = poubelle
            #Stabilité
            s_ind = self.robot_calculStab()
            if s_ind <= 0.5 and s_ind > 0.45:
                score += -dt*50
            elif s_ind > 0.5:
                score += -dt*200
            
            #Fin zone de travail
            
            if self.PatteHD.posStraj[0] == False:
                C_check_1 = True
            if self.PatteBD.posStraj[0] == False:
                C_check_2 = True
            if self.PatteHD.posStraj[3] == True and C_check_1:
                P_check_0 = True
            if self.PatteBD.posStraj[1] == 0 and C_check_2:
                P_check_1 = True
            itera += 1
        temps_traj = itera*dt #Durée de la trajectoire
        
        vit_x =  d_x/temps_traj
        der_y = d_y/temps_traj
        
        #print(temps_traj)
        score += abs(vit_x)*10
        
        return score
    
    def robot_evalTrajRotation(self):
        score = 0
        dt = 0.15
        
        #Analyse n°1 : Détermination de la rotation pour une trajectoire
        traj = self.PatteBD.Straj.getTraj()
        BP_l = self.PatteBD.Straj.getBP()
        posP = np.array(self.PatteBD.Tbase*np.matrix(list(traj[0])+[1]).transpose())[:-1]
        posP = np.array([posP[i] for i in range(len(BP_l)) if BP_l[i] == True]) #On prend que les valeurs où l'effecteur effectue la trajectoire
        posPC = posP[:][0]+1j*posP[:][1] #On passe en complexe
        dposPC = posPC[1:]-posPC[:-1]
        darg=np.angle(dposPC)
        
        arg = np.sum(darg) #Angle effectué lors de la rotation
        
        #
        C_check_1, C_check_2, P_check_0, P_check_1 = False, False, False, False #Pour savoir si l'on a fait un cycle de marche complet, on regarde si la patte en decalage est passe par sa position initial et si l'autre patte (pas décalé) à finit son TN
        max_iter = 600
        itera = 0
        #Initialisation zone de travail
        T = 0
        
        #Boucle principale
        while (not (P_check_0 and P_check_1)) and max_iter > itera:
            self.robot_udpate(dt)
            
            #Zone de travail
            
            T += dt
            
            posHD = self.PatteHD.getPosEffecteur()
            posBD = self.PatteBD.getPosEffecteur()
            
            #Trajectoire valide ?
            if (self.PatteHD.posStraj[0] == False and self.PatteBD.posStraj[0] == True and posHD[2] < posBD[2]) or (self.PatteHD.posStraj[0] == True and self.PatteBD.posStraj[0] == False and posHD[2] > posBD[2]):
                #traj_valide = False
                return -np.inf #Trajectoire non valide = poubelle
            
            s_ind = self.robot_calculStab()
            if s_ind <= 0.5 and s_ind > 0.45:
                score += -dt*50
            elif s_ind > 0.5:
                score += -dt*200
            
            #Fin zone de travail
            
            if self.PatteHD.posStraj[0] == False:
                C_check_1 = True
            if self.PatteBD.posStraj[0] == False:
                C_check_2 = True
            if self.PatteHD.posStraj[3] == True and C_check_1:
                P_check_0 = True
            if self.PatteBD.posStraj[1] == 0 and C_check_2:
                P_check_1 = True
            itera += 1
            
        score += arg/T
        
    def exportTrajBis(self, NomFichier):
        """
        Permet d'exporter la trajectoire suivie au format
        
        t | q1 | q2 | q3 | q4 | q5 | q6 | q7 | q8 | q9 | q10 | q11 | q12

        avec t le temps à chaque point et q1, ..., q12 les position angulaire de chaque moteur

        """
        max_iter = 600
        dt = 0.1
        itera = 0
        R = []
        
        C_check_1, C_check_2, P_check_0, P_check_1 = False, False, False, False #Pour savoir si l'on a fait un cycle de marche complet, on regarde si la patte en decalage est passe par sa position initial et si l'autre patte (pas décalé) à finit son TN
        
        #On effectue une fois toute la trajectoire
        while (not (P_check_0 and P_check_1)) and max_iter > itera:
            self.robot_udpate(dt)
            
            #Zone de travail
            
            R.append([dt*itera]+list([float(x)*180/np.pi for x in self.PatteHD.getPosAngMot()])+list([float(x)*180/np.pi for x in self.PatteHG.getPosAngMot()])+list([x*180/np.pi for x in self.PatteBD.getPosAngMot()])+list([x*180/np.pi for x in self.PatteBG.getPosAngMot()]))
            
            #Fin zone de travail
            
            if self.PatteHD.posStraj[0] == False:
                C_check_1 = True
            if self.PatteBD.posStraj[0] == False:
                C_check_2 = True
            if self.PatteHD.posStraj[3] == True and C_check_1:
                P_check_0 = True
            if self.PatteBD.posStraj[1] == 0 and C_check_2:
                P_check_1 = True
            itera += 1
        with open(NomFichier+".csv", 'w', newline='') as csvFichier:
            ecriture = csv.writer(csvFichier, delimiter=';',quotechar='|', quoting=csv.QUOTE_MINIMAL)
            ecriture.writerows(R)
    
    def robot_setTrajCustom(self, TrajHD:Trajectoire, TrajHG:Trajectoire, TrajBD:Trajectoire, TrajBG:Trajectoire, ret=[-1, -1, -1, -1]):
        """
        Set la trajectoire du robot aux différentes Trajectoire d'entrés
        """
        self.PatteHD.traj_Suivie(TrajHD, False, ret[0])
        self.PatteHG.traj_Suivie(TrajHG, True, ret[1])
        self.PatteBG.traj_Suivie(TrajBG, False, ret[2])
        self.PatteBD.traj_Suivie(TrajBD, True, ret[3])
        
    def robot_udpate_3p(self, dt:float) -> None:
        """
        Fonction d'udpate du robot
        
        Paramètre : 
            dt : l'intervalle de temps entre chaque update
        """
        self.PatteHD.patte_udpate(dt)
        self.PatteHG.patte_udpate(dt)
        self.PatteBG.patte_udpate(dt)
        self.PatteBD.patte_udpate(dt)
        
        self.PatteBG.signal_setRP(self.PatteHD.signal_getTN())
        self.PatteHG.signal_setRP(self.PatteBG.signal_getTN())
        
        self.PatteBD.signal_setRP(self.PatteHG.signal_setRP())
        self.PatteHD.signal_setRP(self.PatteBD.signal_getTN())