# JumeauRobotQuadripede

Ce projet s'inscrit dans un projet de groupe visant à concevoir et à faire réaliser des trajectoires à un robot quadrupède. Mon rôle était de concevoir les trajectoires que le robot devait réaliser, ainsi que les exporter en un format utilisable par le responsable de l'implémentation du code dans le robot.

Cela m'a mener à la réalisation d'un jumeau numérique cinématique d'un robot quadrupède afin de concevoir sa trajectoire et de l'optimiser via un algorithme génétique.

Objectifs : 
- Réaliser un jumeau numérique cinématique du robot capable de réaliser une trajectoire quelquonque
- Être capable d'évaluer les trajectoires
- Optmiser une trajectoire (via algorithme génétique)
- Exporter une trajectoire sous deux format :
  1er format : une liste où les colonnes sont 
  | x | y | z | BP |
  avec x, y, z les positions du i-ème point de passage et BP un booléen identiquant si oui ou non le point est en contact avec le sol

  2eme format : une liste où les colonnes sont 
  t | q_1 | q_2 | q_3 | ... | q_12 |
  avec q_1, ..., q_12 les angles moteurs au temps t

# Résulats 

![til](./Resultats/traj.gif)
<legend>Simulation du robot suivant une trajectoire optimisée (itérations : 300)</legend>

![alt text](https://github.com/KLB-F/ProjetRobotique/blob/ffc319b9bc4cacaf6672e6c26e9feeabc2315b62/Resultats/%20EvolutionVitesseIteration.png)
<legend>Evolution de la vitesse moyenne du robot calculé issuent des 5 meilleurs trajectoirs à la i-ème itérations</legend>
