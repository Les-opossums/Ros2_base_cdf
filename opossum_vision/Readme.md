1.  Votre package s'appelle `opossum_vision`.
2.  Vous avez configuré les points d'entrée (`console_scripts`) dans votre `setup.py` :
      * L'émetteur est accessible via `vision_emulator_node`.
      * L'écouteur est accessible via `vision_node`.

-----

## 1\. Démarrer le Tunnel Série Virtuel (`socat`)

Cette commande doit être lancée en premier, dans un terminal séparé, et rester active. Elle crée une paire de ports virtuels connectés entre eux (le "câble").

```bash
sudo socat -d -d pty,raw,echo=0,link=/dev/ttyV0,mode=777 pty,raw,echo=0,link=/dev/ttyV1,mode=777
```

**ATTENTION :** Notez les deux ports affichés (par exemple `/dev/pts/2` et `/dev/pts/3`). Le premier sera l'émetteur, le second le récepteur.

-----

## 2\. Lancer le Nœud Émetteur (Simulateur)

Ce nœud génère les coordonnées et les envoie sur le port série (le premier de la paire `socat`).

**Hypothèse :** Le port émetteur est `/dev/pts/2`.

```bash
# Lancer le nœud "vision_emulator" en lui donnant le port émetteur
ros2 run opossum_vision vision_emulator_node.py --ros-args -p serial_port:=/dev/ttyV0
```

-----

## 3\. Lancer le Nœud Écouteur (Intégration ROS 2)

Ce nœud lit le port série (le second de la paire `socat`), décode les données et les publie sur un topic ROS 2.

**Hypothèse :** Le port récepteur est `/dev/pts/3`.

```bash
# Lancer le nœud "serial_listener" en lui donnant le port récepteur
ros2 run opossum_vision vision_node.py --ros-args -p serial_port:=/dev/ttyV1
```

-----

## 4\. Vérifier l'Intégration ROS 2

Utilisez cette commande dans un autre terminal pour vous assurer que les données sont bien reçues, décodées et publiées sur le topic ROS 2.

```bash
# Afficher les messages publiés sur le topic /object_pose
ros2 topic echo /object_pose
```

Si tout fonctionne, vous verrez les messages `geometry_msgs/Pose` s'afficher et leurs valeurs changer au rythme de l'émulateur.