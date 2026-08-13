# Checkpoint del modelo

Coloca aquí tu checkpoint entrenado con el nombre `unet_best.pt`
(el mismo `best.pt` que genera `train.py`, renombrado).

Ruta esperada por defecto (ver `config/params.yaml`):

```
lane_detection_cnn/models/unet_best.pt
```

Este archivo **no debe subirse a git** si pesa más de ~50-100 MB — usa Git
LFS, o transfiérelo aparte (scp / Zenodo) entre el MSI Titan (donde
entrenaste) y el Dell (donde vas a correr la inferencia para el benchmark).

Después de colocarlo aquí, `colcon build` lo copia a
`install/lane_detection_cnn/share/lane_detection_cnn/models/unet_best.pt`,
que es donde el nodo lo busca por defecto si `cnn.model_path` es relativo.
