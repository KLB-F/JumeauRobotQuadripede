from ModeleGeometrique import ModeleGeometrique
from Objet import Objet
from Moteur import Moteur
from Trajectoire import Trajectoire

import csv
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

"""
Classe Patte du robot, qui s'occupe de simuler une patte
"""

class Patte(Objet, ModeleGeometrique):
    def __init__(self, l1:float, l2:float, l3:float, Tbase:[[float]*4], m1:Moteur, m2:Moteur, m3:Moteur):
        """
        Initalise la patte

        Parametres
        ----------
        l1, l2, l3 : les longueurs l1, l2, l3 du schéma cinématique
        Tbase : la matrice de transformation homogène du repère de la patte au repère du robot
        m1, m2, m3 : Les moteurs utilisé dans l'articulation
        """
        self.l1, self.l2, self.l3 = l1, l2, l3
        self.Tbase = Tbase
        
        self.M1, self.M2, self.M3 = m1, m2, m3
        
        self.Straj = None #Trajectoire suivie : Aucune au début
        self.posStraj = [True, 0, False, False] #Rendue dans le suivie de la trajectoire
        
        # Inutile a priori #self.q1, self.q2, self.q3 = self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng()
        
        super().__init__(self.l1, self.l2, self.l3) #On initialise le ModeleGeometrique
    
    def getPos(self):
        return self.Tbase
    
    def getPosEffecteurRel(self):
        """
        Renvoie la position de l'effecteur (x, y, z) relatif au repère de la patte
        """
        return self.MGD(self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng())
    
    def getPosEffecteur(self):
        """
        Renvoie la position de l'effecteur (x, y, z) relatif au repère du robot
        """
        return (self.Tbase*np.matrix(list(self.getPosEffecteurRel())+[1]).transpose())[:-1]
    
    def setObjEffecteurRel(self, x, y, z):
        """
        Fixe la position de la patte à atteindre dans son repère
        
        Parametres :
            x, y, z : la position à atteindre
        """
        q1, q2, q3 = self.MGI(x, y, z)
        
        self.M1.set_PosObj(q1)
        self.M2.set_PosObj(q2)
        self.M3.set_PosObj(q3)
        
    def patte_udpate(self, dt):
        """
        Fonction d'udpate de la patte
        
        Paramètres : le pas temporelle dt
        """
        self.M1.mot_update(dt)
        self.M2.mot_update(dt)
        self.M3.mot_update(dt)
        
        #Suivie de trajectoire
        if self.Straj != None:
              if np.linalg.norm(np.array(self.Straj.getTraj()[self.posStraj[1]])-np.array(self.MGD(self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng()))) < 0.01: #Distance < epsilon
                  posFut = self.posStraj[1] + 1
                  if len(self.Straj.getTraj())-1 < posFut:
                      posFut = 0
                  
                  futBP = self.Straj.getBP()[posFut]
                  if futBP == True and self.posStraj[0] == False:
                      self.posStraj[3] = True
                  elif futBP == False and self.posStraj[0] == True:
                      if self.posStraj[2] == False: 
                          return None
                      self.posStraj[2] = False
                      self.posStraj[3] = False
                      
                  self.posStraj[1] = posFut                      
                  self.posStraj[0] = futBP
                  self.setObjEffecteurRel(*self.Straj.getTraj()[self.posStraj[1]])
    
    def patte_udpate_nano(self, dt):
        """
        Fonction d'udpate de la patte
        
        Paramètres : le pas temporelle dt
        """
        self.M1.mot_update(dt)
        self.M2.mot_update(dt)
        self.M3.mot_update(dt)
    
    def getPosAngMot(self) -> [float, float, float]:
        """ Retourne la position angulaire de chaque moteur (q1, q2, q3) """
        return self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng()
    
    def traj_Suivie(self, trajectoire:Trajectoire, decalage=False, ret=-1):
        """
        Permet de faire suivre une trajectoire au Robot
        
        Paramètres : 
            Trajectoire : la trajectoire à suivre
            decalage : permet de fixer s'il y a un décalage
        """
        self.Straj = trajectoire
        
        if decalage == False:
              self.M1.q, self.M2.q, self.M3.q = self.MGI(*self.Straj.getTraj()[0])
              self.setObjEffecteurRel(*self.Straj.getTraj()[0])
              self.posStraj = [True, 0, False, False]
        else:
            if ret != -1:
                self.posStraj = [True, ret+1, False, False]
                self.M1.q, self.M2.q, self.M3.q = self.MGI(*self.Straj.getTraj()[ret])
                self.setObjEffecteurRel(*self.Straj.getTraj()[ret+1])
                return None
            #Trouve le dernier RP
            maxi = 0 #Hypothèse maxi > 0 et maxi < len(self.Straj.getBP())
            for i in range(len(self.Straj.getBP())):
                if self.Straj.getBP()[i] == True:
                    maxi = i
            self.posStraj = [True, 0, False, False]
            self.posStraj[1] = (maxi+1)%(len(self.Straj.getTraj()))
            self.M1.q, self.M2.q, self.M3.q = self.MGI(*self.Straj.getTraj()[maxi])
            self.setObjEffecteurRel(*self.Straj.getTraj()[(maxi+1)%(len(self.Straj.getTraj()))])
            self.posStraj[0] = False
        
    def signal_setRP(self, signal:bool) -> None:
        """ Permet d'indiquer si elle à le droit de commencer son mouvement de RP"""
        self.posStraj[2] = signal
    
    def signal_getTN(self) -> bool:
        """ Indique si la patte à finie son mouvement de TN"""
        return self.posStraj[3]

    def signal_getRP(self) -> bool:
        """ Indique si la patte est en mode RP """
        return self.posStraj[0]        
        
    def patte_exportTraj(self, NomFichier:str):
        """ Permet d'exporter la trajectoire de la patte
        
        Paramètres : 
            - NomFichier : Le nom du fichier
        Retourne : 
            - Un fichier csv portant le nom donné et contenant les information de la trajectoire
        """
        R = []
        for i in range(len(self.Straj.getTraj())):
            R.append(list(self.MGI(*self.Straj.getTraj()[i]))+[self.Straj.getBP()[i]])
        
        with open(NomFichier+".csv", 'w', newline='') as csvFichier:
            ecriture = csv.writer(csvFichier, delimiter=';',quotechar='|', quoting=csv.QUOTE_MINIMAL)
            ecriture.writerows(R)
            
    def anim_traj(self):
        """
        Affiche une animation de la patte réalisant la trajectoire actuelle
        """
        dt = 0.03
        ax_echelle = 2.5
        x_p, y_p, z_p = [], [], []
        for i in range(600):
            self.posStraj[2] = True
            self.posStraj[3] = True
            
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            
            xsT, ysT, zsT = [], [], []
            for k in range(len(self.Straj.getTraj())):
                xsT.append(self.Straj.getTraj()[k][0])
                ysT.append(self.Straj.getTraj()[k][1])
                zsT.append(self.Straj.getTraj()[k][2])
            
            X_trace = self.X_trace(self.M1.get_PosAng(), self.M2.get_PosAng(), self.M3.get_PosAng())        
            # conversion en coordonnées x,y,z pour matplotlib
            xs = [float(p[0]) for p in X_trace]
            ys = [float(p[1]) for p in X_trace]
            zs = [float(p[2]) for p in X_trace]
            
            # Visualisation
            ax.plot(xs, ys, zs, '-k', linewidth=3, markersize=6)
            ax.plot(xs, ys, zs, 'or', linewidth=3, markersize=6)
            ax.plot(x_p, y_p, z_p, '-b', linewidth=3, markersize=6, alpha=0.5)
            ax.plot(xsT, ysT, zsT, 'bv', linewidth=3, markersize=6, alpha=0.5)
            
            ax.view_init(elev=25, azim=45) # réglages de la vue 3D
            
            ax.axes.set_xlim3d(left=-ax_echelle, right=ax_echelle)
            ax.axes.set_ylim3d(bottom=-ax_echelle, top=ax_echelle)
            ax.axes.set_zlim3d(bottom=-ax_echelle, top=ax_echelle)
            ax.set_xlabel("X (dm)")
            ax.set_ylabel("Y (dm)")
            ax.set_zlabel("Z (dm)")
            
            
            plt.savefig("temp/"+str(i)+".png")
            plt.close(fig)
            
            x_p.append(xs[-1])
            y_p.append(ys[-1])
            z_p.append(zs[-1])
            self.patte_udpate(dt)
        images = [Image.open("temp/"+str(i)+".png") for i in range(300)]
        images[0].save("traj.gif", save_all=True, append_images=images[1:], duration=0.3*100, loop=1)

    def load_Traj(self, nomFichier:str):
            
        pt = []
        BP = []
            
        with open(nomFichier+".csv", 'r', newline='') as csvFichier:
            reader = csv.reader(csvFichier, quoting=csv.QUOTE_NONE)
                
            for row in reader:
                l = row[0].split(";")
                q = l[:-1]
                bp = l[-1]
                
                if bp == "True":
                    BP.append(True)
                else:
                    BP.append(False)
                pt.append(self.MGD(*[float(a) for a in q]))
        return Trajectoire(pt, BP)
    
    def TraceVolume(self):
        qpos = []
        for q1 in range(-int(np.pi/2*100), int(np.pi/2*100), 100):
            for q2 in range(-int(np.pi/2*100), int(np.pi/2*100), 100):
                    qpos.append([q1, q2])
        
        min0 = 0
        for i in range(len(qpos)):
            if abs(qpos[i][0]) < abs(qpos[min0][0]):
                min0 = i
        v0 = self.MGD(*(qpos[min0]+[0]))
        
        a = [e for e in qpos if (self.MGD(*(e+[0]))[0] - v0[0]) < 0.005 and qpos[min0][1] == e[1]]
        return a
        