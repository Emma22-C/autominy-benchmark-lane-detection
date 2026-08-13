# lane_detection_cnn

Cuarto detector de carril del benchmark (U-Net sobre imagen IPM), completando
la simetría con `lane_detection_fcm`, `lane_detection_sliding_windows` y
`lane_detection_hough`. Publica el mismo contrato de topics que esos tres
nodos, para que `metrics_collector_node` lo consuma sin cambios.

## Por qué Python (rclpy) y no C++ (LibTorch)

Los otros tres nodos son C++ y usan `lane_common::Preprocessor` directamente.
Este nodo usa **rclpy + PyTorch nativo** porque:

- El checkpoint (`unet_best.pt`) ya está en formato nativo PyTorch
  (`state_dict`), no en TorchScript — usarlo desde C++ requeriría exportarlo
  primero (`torch.jit.trace`/`script`) y enlazar LibTorch en el build, con
  el riesgo de que las versiones de CUDA/cuDNN entre LibTorch y el PyTorch
  con el que entrenaste no coincidan exactamente.
- El paso de IPM se **reimplementó en Python** (`preprocessing.py`) usando
  los mismos parámetros calibrados, en vez de hacer binding a la clase C++.
  Geométricamente debería dar el mismo resultado, pero **no se verificó
  pixel a pixel contra `lane_common::Preprocessor`** — ver la nota al inicio
  de `preprocessing.py` sobre cómo comparar `~/debug_image` de este nodo
  contra el de `lane_detection_fcm` con el mismo frame antes de confiar en
  los números para el paper.

Si prefieres paridad completa en C++/LibTorch en vez de esta ruta, es
factible pero añade bastante complejidad de build — dime y lo evaluamos.

## Entorno Python (punto crítico)

Entrenaste la CNN en un entorno **conda** (`lane_cnn`) en el MSI Titan. ROS2
Humble típicamente usa el **Python del sistema** (`/usr/bin/python3`), no
conda. Si activas el entorno conda y luego haces `source install/setup.bash`,
es común que `rclpy` deje de encontrarse (choque de intérpretes).

Recomendación: **no mezcles conda con el workspace ROS2**. Instala
`torch` + `opencv-python` directamente en el Python del sistema que usa
colcon:

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121 \
    --break-system-packages   # o el flag equivalente en tu distro
pip install opencv-python --break-system-packages
```

Verifica antes de compilar:

```bash
python3 -c "import rclpy, torch, cv2; print('OK', torch.__version__, torch.cuda.is_available())"
```

## Compilar

1. Copia tu checkpoint a `models/unet_best.pt` (ver `models/README.md`).
2. Desde la raíz del workspace:

```bash
colcon build --packages-select lane_detection_cnn --symlink-install
source install/setup.bash
```

## Correr

```bash
ros2 launch lane_detection_cnn lane_detection_cnn.launch.py
```

Verifica los topics:

```bash
ros2 topic list | grep lane_detection_cnn
ros2 topic echo /lane_detection_cnn/center_deviation
```

## Pendiente antes de usarlo para el benchmark final

- [ ] Confirmar `val_iou` del checkpoint (para reportarlo en el paper) —
      corre: `python3 -c "import torch; print(torch.load('models/unet_best.pt', map_location='cpu').get('val_iou'))"`
- [ ] Verificar que `config/params.yaml` coincide EXACTAMENTE con el YAML
      real de `lane_detection_fcm` en `preprocessing.*` y
      `camera_center_offset` (los valores aquí vienen de la memoria del
      proyecto, no se confirmaron contra el YAML real).
- [ ] Comparar `~/debug_image` de este nodo contra `lane_detection_fcm` en el
      mismo frame para validar que la IPM en Python coincide con la de
      `lane_common::Preprocessor` en C++.
- [ ] Correr sobre `tmr_vuelta1` y generar el CSV con el mismo esquema de 14
      columnas (`xie_beni_*`/`fpc_*` como NaN para `algo=cnn`).
