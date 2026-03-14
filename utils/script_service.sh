#!/bin/bash

# 1. Sourcer l'environnement ROS 2 (à adapter selon votre version, ex: humble, foxy...)
source /opt/ros/humble/setup.bash
source /home/opossum/robot_ws/install/setup.bash

# 2. Définir le chemin du fichier d'état
CONFIG_FILE="/home/opossum/robot_ws/config_plateau.txt"

# 3. Lire la configuration (par défaut "petit" si le fichier n'existe pas)
if [ -f "$CONFIG_FILE" ]; then
    CHOIX=$(cat "$CONFIG_FILE")
else
    CHOIX="petit"
fi

# 4. Lancer le bon launchfile selon le choix
echo "Lancement de la configuration : $CHOIX"

if [ "$CHOIX" == "petit" ]; then
    exec ros2 launch opossum_bringup bringup_small_area.launch.py
elif [ "$CHOIX" == "grand" ]; then
    exec ros2 launch opossum_bringup bringup_simu.launch.py
else
    echo "Erreur : Configuration '$CHOIX' inconnue."
    exit 1
fi
