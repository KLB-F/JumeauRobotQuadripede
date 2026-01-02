"""
Classe décrivant le fonctionnement d'un moteur
Elle simule le comportement d'un moteur auto-asservie parfait

Unités :
    Angle : rad
    Vitesse : rad/s

"""

class Moteur():
    def __init__(self, q0, qmin, qmax, qvit):
        """
        Initialise le moteur

        Parameters
        ----------
        q0 : La position initial du moteur
        qmin : la position minimal que peut prendre le moteur
        qmax : la position maximal que peut prendre le moteur
        qvit : la vitesse angulaire du moteur 
        """
        self.q = q0
        self.qmin, self.qmax = qmin, qmax
        self.q_vit = qvit
        self.q_obj = q0
    
    def mot_update(self, dt):
        """
        Fonction d'udpate du moteur
        
        Paramètres : dt le pas temporelle
        """
        if (self.q_obj-self.q) > self.q_vit*dt:
            self.q = self.q + self.q_vit*dt
        elif (self.q_obj-self.q) < -self.q_vit*dt:
            self.q = self.q - self.q_vit*dt
        else:
            self.q = self.q_obj
    
    def set_PosObj(self, q_obj):
        """
        Fonction permettant de fixer l'angle à atteindre
        """
        if q_obj > self.qmax:
            self.q_obj = self.qmax
        elif q_obj < self.qmin:
            self.q_obj = self.qmin
        else:
            self.q_obj = q_obj
            
    def get_PosObj(self):
        return self.q_obj
    
    def get_PosAng(self):
        """
        Fonction donnant la position angulaire du moteur
        """
        return self.q
    
    def get_Param(self):
        """
        Retourne les paramètres du moteur (qmin, qmax, qvit)
        """
        return self.qmin, self.qmax, self.q_vit
            