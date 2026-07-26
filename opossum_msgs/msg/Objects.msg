int16 id
string type
string state # If free, pos world else pos in robot
float32 x
float32 y
float32 theta

# --- Fusion / confiance (rempli par opossum_vision/tag_fusion_node) ---
# confidence : confiance [0..1] dans la position. Haute a l'arret, degradee
#   en mouvement (surtout en rotation) et decroissante avec l'age.
# age_s      : temps (s) depuis la derniere observation reelle de l'objet.
# present    : true tant que l'objet est consideré present (age < seuil) ;
#   false = persistance (memorise mais plus vu recemment).
float32 confidence
float32 age_s
bool present
