from abc import ABC, abstractclassmethod

"""
Classe abstraite Objet, qui concerne tous les objets physique
"""

class Objet():
    @abstractclassmethod
    def getPos(self) -> [float, float, float]:
        """
        Renvoie la position de l'objet
        """
        pass