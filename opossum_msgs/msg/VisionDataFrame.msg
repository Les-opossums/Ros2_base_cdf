int32 id

# Instant de capture de la frame (secondes, horloge du Raspberry Pi/ROS,
# recalee depuis l'horloge locale du JeVois via les trames HEARTBEAT --
# voir vision_node.py). Vaut 0.0 si non disponible (camera pas encore
# calibree, ou source ne fournissant pas ce champ, ex: simulateur), auquel
# cas le consommateur doit se rabattre sur la pose robot courante.
float64 capture_time

opossum_msgs/VisionData[] object
