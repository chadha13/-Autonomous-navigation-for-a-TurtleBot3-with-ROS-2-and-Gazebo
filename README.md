# TurtleBot3 : navigation autonome A* + PID (ROS 2 Humble, Gazebo)

Un TurtleBot3 Waffle trouve tout seul le **plus court chemin** vers un point cible dans une maison simulée sous Gazebo, puis le suit avec des **régulateurs PID**.

- **Carte** : construite avec le SLAM (Cartographer), sauvegardée dans `house_map.yaml` / `house_map.pgm`
- **Localisation** : AMCL (Nav2) place le robot sur la carte
- **Planification** : A* en 8 directions, obstacles gonflés pour garder une marge de sécurité
- **Contrôle** : PID sur le cap (vitesse angulaire) et sur la distance (vitesse linéaire), commande publiée sur `/cmd_vel` à 10 Hz

> Vidéo de démonstration : _[ajouter le lien LinkedIn / YouTube]_

## Fonctionnement

```
 /scan, /odom ──► AMCL ──► TF map → base_footprint ──┐
                                                      ▼
 house_map.yaml ──► A* (plus court chemin) ──► waypoints ──► PID ──► /cmd_vel ──► TurtleBot3
                          │
                          └──► /planned_path (visible dans RViz)
```

1. Le nœud lit la pose du robot dans la carte (TF `map → base_footprint`, fournie par AMCL).
2. A* calcule le chemin le plus court entre la pose du robot et le but, sur la carte d'occupation gonflée.
3. Le chemin est simplifié en quelques waypoints (suppression des points alignés grâce à un test de visibilité).
4. À 10 Hz, deux PID suivent le waypoint courant :
   - `w = PID(e_yaw)` avec `e_yaw = atan2(dy, dx) - yaw` ramené dans [−π, π]
   - `v = PID(dist)` sur le dernier waypoint, vitesse de croisière entre les waypoints, atténuée par `max(0, cos(e_yaw))` pour pivoter avant d'avancer
5. À l'arrivée (`dist ≤ tol_pos`), le robot s'arrête.

## Prérequis

- Ubuntu 22.04, ROS 2 Humble
- Gazebo, TurtleBot3 (`turtlebot3_gazebo`, `turtlebot3_navigation2`, `turtlebot3_cartographer`)
- Nav2 (`nav2_bringup`, `nav2_map_server`)
- Python 3 avec `numpy` et `pyyaml` (déjà présents avec ROS 2 Humble)

```bash
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=30
```

## Lancer la démonstration

Un terminal par bloc, après `source /opt/ros/humble/setup.bash` et `source ~/turtlebot3_ws/install/setup.bash`.

**1. Gazebo (monde maison)**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**2. Carte + AMCL + RViz**
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/house_map.yaml
```

**3. Dans RViz**
- Cliquer sur **2D Pose Estimate**, puis sur la carte à la position du robot (faire glisser pour donner le cap).
- Tourner un peu le robot (`teleop_keyboard`) jusqu'à ce que le laser coïncide avec les murs.
- Optionnel : `Add → By topic → /planned_path → Path` pour voir le chemin calculé.
- Ne pas utiliser « Nav2 Goal ».

**4. Navigation A\* + PID**
```bash
python3 astar_pid_nav.py --ros-args \
  -p use_sim_time:=true -p use_tf:=true -p unknown_free:=true \
  -p map_file:=$HOME/house_map.yaml \
  -p goal_x:=6.0 -p goal_y:=2.0 -p v_max:=0.25 -p w_max:=1.5
```

## Paramètres

| Paramètre | Défaut | Rôle |
|---|---|---|
| `goal_x`, `goal_y` | 2.0, 0.0 | but, en mètres, dans le repère de la carte |
| `map_file` | `~/house_map.yaml` | carte à utiliser |
| `use_tf` | false | lit la pose via TF (AMCL). Sinon, utilise `/odom` + `init_x/y/yaw` |
| `unknown_free` | false | zones non explorées traversables (coût × `unknown_cost`) |
| `unknown_cost` | 3.0 | pénalité pour traverser une zone inconnue |
| `inflate_radius` | 0.28 | marge de sécurité autour des murs (m) |
| `v_max`, `w_max` | 0.22, 1.2 | vitesses maximales (m/s, rad/s) |
| `kp_lin`, `ki_lin`, `kd_lin` | 0.8, 0, 0.05 | PID de distance |
| `kp_ang`, `ki_ang`, `kd_ang` | 2.0, 0, 0.1 | PID de cap |
| `tol_pos`, `wp_tol` | 0.10, 0.20 | tolérance d'arrivée au but / aux waypoints intermédiaires |

## Créer sa propre carte

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
ros2 run turtlebot3_teleop teleop_keyboard      # explorer toute la maison, lentement
ros2 run nav2_map_server map_saver_cli -f ~/house_map
```

Explorer toutes les pièces avant de sauvegarder : les zones grises (inconnues) ne sont pas fiables pour la planification.

## Contenu du dépôt

```
astar_pid_nav.py   nœud ROS 2 : chargement de la carte, A*, PID, publication /cmd_vel
house_map.yaml     métadonnées de la carte (résolution, origine)
house_map.pgm      carte d'occupation
```

## Limites

- Le chemin est calculé **une seule fois** sur une carte statique : pas d'évitement d'obstacles dynamiques ni de replanification.
- Avec `unknown_free:=true`, le robot peut traverser des zones non explorées (pénalisées) où des obstacles ne sont pas connus.
- Le suivi se fait waypoint par waypoint, sans lissage de trajectoire : à grande vitesse, le robot peut frôler les murs dans les virages serrés.

## Pistes d'amélioration

- Évitement d'obstacles avec `/scan` et replanification
- Lissage de trajectoire (splines, pure pursuit)
- Comparaison avec le planificateur Nav2

## Crédits

Basé sur le TP 2 « TurtleBot3 » de Sarra Ben Halima (contrôle proportionnel vers un point), étendu avec la planification A*, les PID et la localisation AMCL.

## Licence

_[à choisir, par exemple MIT]_
