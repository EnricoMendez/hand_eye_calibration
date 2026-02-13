# hand_eye_calibration

Paquete ROS 2 para calibración eye-in-hand con Lite6 + RealSense + ArUco.

## Resumen
Este paquete incluye:

- Detección y estimación de pose de marcador ArUco (`aruco_pose_estimation`).
- Nodo de calibración hand-eye interactivo (`hand_eye_calibrator`).
- Launch unificado para levantar robot, cámara, calibración y RViz (`lite6_hand_eye_calibration.launch.py`).
- Nodo de movimiento aleatorio del robot condicionado a visibilidad del marcador (`random_marker_motion`).

## Nodos

### `aruco_pose_estimation`
- Suscribe imagen y `camera_info`.
- Detecta marcadores ArUco y publica:
  - `detection` (`sensor_msgs/Image`)
  - `marker_poses` (`geometry_msgs/PoseArray`)
- Compatible con APIs antiguas y nuevas de OpenCV ArUco.
- Permite filtrar por ID de marcador desde el constructor (`target_marker_id`):
  - `None`: procesa todos.
  - entero: procesa solo ese ID.

### `hand_eye_calibrator`
- Toma muestras para calibración eye-in-hand usando:
  - `T_base_ee` desde TF (`base_frame` -> `ee_frame`)
  - `T_camera_marker` desde `marker_poses`
- Calcula calibración con `cv2.calibrateHandEye`.
- Publica:
  - TF estático final `ee_frame -> result_child_frame`
  - TF dinámicos en `/tf` (`tf2_msgs/msg/TFMessage`):
    - `base_frame -> camera_estimated_frame`
    - `base_frame -> marker_estimated_frame` (si hay marcador)
  - `MarkerArray` en `viz_topic` (ejes de EE, cámara estimada y marcador estimado).

Controles de teclado en terminal:
- `s`: capturar muestra
- `c`: calcular calibración
- `w`: guardar resultado JSON
- `r`: reset de muestras
- `q`: salir

Servicios disponibles (`std_srvs/Trigger`):
- `/capture_sample`
- `/compute_calibration`
- `/save_calibration`
- `/reset_samples`

### `random_marker_motion`
- Mueve Lite6 a posiciones aleatorias dentro de rangos configurables.
- Usa servicios de `xarm_api` (`set_position`, `get_position`, `set_mode`, `set_state`, `motion_enable`).
- Solo mueve cuando el marcador está visible recientemente en `/marker_poses`.

## Launch principal

Archivo:
- `launch/lite6_hand_eye_calibration.launch.py`

Incluye:
- `xarm_controller` (ros2_control del Lite6)
- `realsense2_camera` (`rs_launch.py`)
- `aruco_pose_estimation`
- `hand_eye_calibrator`
- RViz con configuración personalizada:
  - `Config/Calibration_visual.rviz`

Comando recomendado:

```bash
ros2 launch hand_eye_calibration lite6_hand_eye_calibration.launch.py robot_ip:=192.168.0.12 add_gripper:=true
```

## Comandos rápidos

Levantar todo (robot + cámara + detección + calibrador + RViz):

```bash
ros2 launch hand_eye_calibration lite6_hand_eye_calibration.launch.py \
  robot_ip:=192.168.0.12 add_gripper:=true
```

Levantar solo detector de pose ArUco:

```bash
ros2 run hand_eye_calibration aruco_pose_estimation --ros-args \
  -p marker_length:=0.075 \
  -p target_marker_id:=12
```

Levantar solo calibrador hand-eye:

```bash
ros2 run hand_eye_calibration hand_eye_calibrator --ros-args \
  -p marker_topic:=/marker_poses \
  -p min_samples:=15 \
  -p base_frame:=link_base \
  -p ee_frame:=link_eef \
  -p camera_frame:=camera_color_optical_frame
```

Servicios (alternativa a teclado):

```bash
ros2 service call /capture_sample std_srvs/srv/Trigger "{}"
ros2 service call /compute_calibration std_srvs/srv/Trigger "{}"
ros2 service call /save_calibration std_srvs/srv/Trigger "{}"
ros2 service call /reset_samples std_srvs/srv/Trigger "{}"
```

Teclas en terminal del calibrador:

```text
s: capturar muestra
c: calcular calibración
w: guardar JSON
r: reset de muestras
q: salir
```

Notas:
- Los logs de RealSense se silencian en pantalla (`output:=log`, `log_level:=error`).
- Los logs de `aruco_pose_estimation` también se reducen para no ensuciar terminal.

## Parámetros útiles del launch

- `robot_ip` (obligatorio para robot real)
- `add_gripper` (`true/false`)
- `start_controller` (default `true`)
- `start_rviz` (default `true`)
- `start_realsense` (default `true`)
- `realsense_log_level` (default `error`)
- `min_samples` (default `15`)
- `marker_length` (default `0.075`)
- `target_marker_id` (default `12`, usar `<0` para procesar todos)

## Visualización en RViz

- Fixed Frame sugerido: `link_base`
- Para ver marcos TF estimados:
  - agregar display `TF`
- Para ver ejes visuales:
  - agregar display `MarkerArray`
  - topic: `/hand_eye_calibration/axes`

## Build

```bash
cd ~/dev_ws
colcon build --packages-select hand_eye_calibration
source install/setup.bash
```

## Ejecución de movimiento aleatorio

Ejemplo:

```bash
ros2 run hand_eye_calibration random_marker_motion --ros-args \
  -p hw_ns:=ufactory \
  -p marker_topic:=/marker_poses \
  -p max_moves:=40 \
  -p range_x:=0.05 -p range_y:=0.05 -p range_z:=0.03 \
  -p range_roll:=0.15 -p range_pitch:=0.15 -p range_yaw:=0.25 \
  -p speed:=80.0 -p acc:=400.0
```

## Archivo de salida de calibración

Por defecto:
- `Config/hand_eye_calibration_result.json` dentro del paquete (si es escribible)
- fallback: `/tmp/hand_eye_calibration_result.json` cuando el paquete instalado no permite escritura

Incluye:
- `camera_to_gripper`
- `gripper_to_camera`
- método usado
- cantidad de muestras
- frames usados
