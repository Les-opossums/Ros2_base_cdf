# opossum_vision

Détection et fusion des tags ArUco des objets de jeu.

## Pipeline

```
JeVois (x3)  --série-->  vision_node.py  --/aruco_loc-->  tag_fusion_node.py  -->  /aruco_world
 (tags dans le repère                     (VisionDataFrame)                    -->  /aruco_world_fused
  ROBOT + capture_time)                                                             (GlobalView)
```

- **`vision_node.py`** — un nœud par caméra. Lit le port série du module JeVois
  (trames binaires ou ASCII), décode les tags ArUco (déjà exprimés **dans le
  repère du robot**) et publie `aruco_loc` (`VisionDataFrame`). Cale aussi
  l'horloge JeVois→Pi via les trames `HEARTBEAT` pour horodater la capture
  (`capture_time`), ce qui permet de compenser le retard caméra en aval.
- **`tag_fusion_node.py`** — consomme `aruco_loc` + `robot_data` (pose/vitesses)
  et publie :
  - `aruco_world` : détections **brutes** recalées en repère monde, à chaque
    trame (visualisation) ;
  - `aruco_world_fused` : estimation **stable et filtrée** par tag, à 10 Hz
    (utilisable en match comme en debug).

Toute la logique de transformation/fusion est dans `opossum_vision/tag_fusion.py`
(bibliothèque pure, testée hors ROS — voir `test/test_tag_fusion.py`).

## Filtrage & confiance

La fusion associe les observations aux objets mémorisés (Hungarian, jamais
entre couleurs ArUco différentes), lisse la position (EMA) et calcule une
**confiance [0..1]** par objet :

- **maximale à l'arrêt** ;
- **dégradée en mouvement**, et **surtout en rotation** (`rot_penalty`) : c'est
  le pire cas pour projeter la position d'un tag vu du robot ;
- **décroissante avec l'âge** (constante `conf_tau_s`) quand l'objet n'est plus
  vu.

En mouvement, l'observation ne déplace que faiblement l'estimé (on fait
confiance à la mémoire, pas à l'observation bruitée).

**Persistance** : un objet non revu reste mémorisé et publié (confiance qui
décroît) ; il est marqué non-présent après `absent_s`, puis oublié après
`forget_s`.

Ces informations sont publiées dans `Objects` : `confidence`, `age_s`,
`present` (+ un résumé lisible dans `state`). L'IHM web les visualise
(opacité/anneau selon la confiance, estompage avec l'âge).

## Paramètres

- Caméras : `config/vision_params.yaml` (`port`, `baudrate`, `camera_id`,
  `binary_input`, `simulation`).
- Fusion : `config/tag_fusion_params.yaml` (latence caméra, seuils
  d'association, modèle de confiance, persistance). Tous réglables à chaud via
  `ros2 param set /main_robot/tag_fusion_node <param> <valeur>`.

## Lancement

```bash
ros2 launch opossum_vision vision.launch.py        # les 3 caméras
ros2 launch opossum_vision tag_fusion.launch.py    # la fusion
```

(Lancés automatiquement par les bringups `opossum_bringup`.)

## Tests

```bash
colcon test --packages-select opossum_vision
```
