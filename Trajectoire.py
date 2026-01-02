"""
Objet Trajectoire : servant à définir une trajectoire
"""

import numpy as np
import matplotlib.pyplot as Picasso
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import newton

import csv

class Trajectoire():
    def __init__(self, PointPassages:[np.array([float, float, float])], BP:[bool] ,d:float=1):
        """ 
        Paramètres : 
            PointPassages : Les points de passages de la trajectoire
            BP ; Tableaux indiquant si, au point de passage associé, le robot est en contact avec le sol
            d : la distance entre chaque points de la trajectoire
        """
        
        self.trajectoire = PointPassages
        self.BP = BP
        if self.BP[0] != True:
            print("Attention : Trajectoire invalide")
        
        """
        for i in range(len(PointPassages)-2):
            tPlus = self.__InterQuadraPoints(PointPassages[i], PointPassages[i+1], PointPassages[i+2], d)
            self.trajectoire = self.trajectoire + tPlus
            if BP[i] == True and BP[i+1] == True:
                self.BP = self.BP + [True]*len(tPlus)
            else:
                self.BP = self.BP + [False]*len(tPlus)"""
                
    
    @staticmethod
    def __InterQuadraPoints(A:np.array, B:np.array, C:np.array, d:float) -> [np.array([float, float, float])]:
        """
        Génère des points toutes les d distances entre les points A, B par interpolation quadriatique sur A, B, C

        paramètres
        ----------
        A, B, C des points de R^3, A != C; B non aligné avec A et C
        d : float
            Distance entre 2 points
        
        Retourne
        -------
        Traj : la liste des points de la trajectoire entre A et B interpolé 

        """
        
        Traj = [A]
        
        #Hypothèse : C != A -> Sinon : Erreur | :(
        e1 = np.array(C-A)/np.linalg.norm(C-A) #Calcul du vecteur directeur de (AC)
        #Point A, B, C alignés ?
        if np.linalg.norm((B-np.array([B[i]*e1[i] for i in range(len(e1))]))) < 0.00001:
            pass
        
        e2 = (B-np.array([B[i]*e1[i] for i in range(len(e1))]))/np.linalg.norm((B-np.array([B[i]*e1[i] for i in range(len(e1))]))) #Calcul de la projection ortohogonal de B sur (AC)
        
        #Remarquons que e1, e2 forment une base orthonormé (merci Gram-Schmit) d'un espace euclidien à 2 dimensions, l'on peut donc y faire un petit polyfit
        coefDir = np.dot(B, e2)/(np.dot(B, e1)-np.dot(A, e1))/(np.dot(B, e1)-np.dot(C, e1))
        Pcoef =  coefDir, -coefDir*(np.dot(A, e1)+np.dot(C, e1)), coefDir*np.dot(A, e1)*np.dot(C, e1)  #np.polyfit([np.dot(A,e1), np.dot(B,e1), np.dot(C,e1)], [0, np.dot(B,e2), 0], 2)
        
        #print(" P.S : ", np.dot(A, e1), np.dot(B, e1), np.dot(C, e1))
        #print(" Vecteurs : ", e1, e2)
        #print(" Pcoef : ", *Pcoef)
        
        def P(a, b, c, x):
            return a*x**2+b*x+c
        
        def dP(a, b, c, x):
            return 2*x*a+b
        
        Picasso.clf()
        X = np.linspace(np.dot(A, e1), np.dot(C, e1), 500)
        Picasso.plot(X, [P(*Pcoef, x) for x in X])
        
        x0 = np.dot(A, e1)
        
        def f(Pcoef,x, x0, d):
            return (P(*Pcoef, x)-P(*Pcoef, x0))**2-(x-x0)**2-d**2
        
        print("D : ", np.linalg.norm(B-Traj[-1]))
        itermax = 20
        i = 0
        while np.linalg.norm(B-Traj[-1]) > d and x0 < np.dot(B, e1) and i < itermax:
            i += 1
            
            x = x0
            for _ in range(itermax):
                fx = f(Pcoef, x, x0, d)
                dfx = dP(*Pcoef,x)
                if dfx == 0:
                    raise ValueError("Dérivée nulle, Newton échoue.")
                x_new = x - fx / dfx
                if abs(fx) < 0.0001:
                    break;
            x0 = x_new
            
            Traj.append(P(*Pcoef, x0)*e2+e1*x0)            
        
        Picasso.legend()
        Picasso.show()
        
        print("df :", np.linalg.norm(B-Traj[-1]))
        return Traj
    
    def getTraj(self):
        return self.trajectoire
    
    def getBP(self):
        return self.BP
    
    def aff_Traj(self):
        """
        Affiche la trajectoire
        """
        xs, ys, zs = [], [], []
        for i in range(len(self.trajectoire)):
            xs.append(self.trajectoire[i][0])
            ys.append(self.trajectoire[i][1])
            zs.append(self.trajectoire[i][2])
        
        Picasso.clf()
        
        fig = Picasso.figure()
        ax = Axes3D(fig, auto_add_to_figure=False)
        fig.add_axes(ax)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        
        ax.plot(xs, ys, zs, '-k', linewidth=3, markersize=6)
        ax.plot(xs, ys, zs, 'or', linewidth=3, markersize=6)

    
    