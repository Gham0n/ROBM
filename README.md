# ROBM

Compte rendu de nos TPs de ROBM

## Authors

- [Hamon Guillaume](https://github.com/Gham0n/tpjpa2023)
- [Léo Nolière](https://github.com/LeoNoliere)

## TP1

Pour le tp1, nous avons construit notre robot et nous avons tester le capteurs couleurs, nous recevions bien les bonnes données RGB des couleurs.

## TP2

Pour le tp2, nous avons fais les 3 scripts de base demandés. Il y a `avance`, `droite` et `tourne_gauche`.

![Video avance](https://github.com/Gham0n/ROBM/blob/main/Videos/avance.mp4)

![Video droite](https://github.com/Gham0n/ROBM/blob/main/Videos/droite.mp4)

![Video tourne gauche](https://github.com/Gham0n/ROBM/blob/main/Videos/tourne_gauche.mp4)

Nous avons également notre script `vel_to_motor` qui nous permet de controler notre robot avec les fleches du pc, et avce la souris.

![Video controle des fleches](https://github.com/Gham0n/ROBM/blob/main/Videos/vel_to_motor.mp4)
![Graphe key](https://github.com/Gham0n/ROBM/blob/main/Photos/graph%20key.PNG)

## TP3

Pour ce 3e TP nous avons implémenté le script `sensor_based_control.py` qui utilise le télémêtre, le contrôle des roues, et la detection des couleurs en simultané. Nous avons ainsi codé le robot pour avoir le comportement suivant :

- Sol **rouge** : vitesse haute (x 2)
- Sol **vert** : vitesse basse (x 0.5)
- Sol **Bleu** : tourner à gauche
- Sol **noir** : Arret du moteur.

Les autres couleurs n'influent pas sur le comportement du robot qui avance alors à la vitesse par défaut définie

Concernant le télémètre celui-ci a aussi une influence sur le comportement des roues, selon les indication suivante

- à moins de 20 cm d'un obstacle on recule.
- quand un obstacle est entre 20cm et 50cm on tourne à droite.
- à plus de 50cm on avance normalement.

## Difficultées rencontrées

Hormis un souci de VM Linux au début du TP1, nous n'avons pas rencontré de difficulté particulière.
