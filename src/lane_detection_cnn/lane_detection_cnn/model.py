"""
model.py

Small U-Net para segmentación binaria de carril.

Misma arquitectura usada en el entrenamiento en el MSI Titan (base Ronneberger
et al. 2015, canales reducidos [16, 32, 64, 128, 256], ~2M parámetros).

IMPORTANTE: esta clase debe coincidir EXACTAMENTE con la usada al generar
unet_best.pt, o load_state_dict() fallará por mismatch de shapes.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


class DoubleConv(nn.Module):
    """(Conv 3x3 -> BatchNorm -> ReLU) x 2."""

    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.block = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.block(x)


class SmallUNet(nn.Module):
    """
    Input:  (B, in_channels, H, W)  -- típicamente (B, 1, 240, 320)
    Output: (B, out_channels, H, W) -- logits crudos, aplicar sigmoid para prob.
    """

    def __init__(self, in_channels=1, out_channels=1, base_channels=16):
        super().__init__()
        c = base_channels

        self.enc1 = DoubleConv(in_channels, c)
        self.enc2 = DoubleConv(c, c * 2)
        self.enc3 = DoubleConv(c * 2, c * 4)
        self.enc4 = DoubleConv(c * 4, c * 8)

        self.bottleneck = DoubleConv(c * 8, c * 16)

        self.dec4 = DoubleConv(c * 16 + c * 8, c * 8)
        self.dec3 = DoubleConv(c * 8 + c * 4, c * 4)
        self.dec2 = DoubleConv(c * 4 + c * 2, c * 2)
        self.dec1 = DoubleConv(c * 2 + c, c)

        self.out_conv = nn.Conv2d(c, out_channels, 1)
        self.pool = nn.MaxPool2d(2)

    def forward(self, x):
        e1 = self.enc1(x)
        e2 = self.enc2(self.pool(e1))
        e3 = self.enc3(self.pool(e2))
        e4 = self.enc4(self.pool(e3))
        b = self.bottleneck(self.pool(e4))

        d4 = F.interpolate(b, size=e4.shape[-2:], mode="bilinear", align_corners=False)
        d4 = self.dec4(torch.cat([d4, e4], dim=1))

        d3 = F.interpolate(d4, size=e3.shape[-2:], mode="bilinear", align_corners=False)
        d3 = self.dec3(torch.cat([d3, e3], dim=1))

        d2 = F.interpolate(d3, size=e2.shape[-2:], mode="bilinear", align_corners=False)
        d2 = self.dec2(torch.cat([d2, e2], dim=1))

        d1 = F.interpolate(d2, size=e1.shape[-2:], mode="bilinear", align_corners=False)
        d1 = self.dec1(torch.cat([d1, e1], dim=1))

        return self.out_conv(d1)


def load_unet(checkpoint_path, base_channels=16, device="cpu"):
    """Carga un checkpoint guardado por train.py (dict con 'model_state')."""
    checkpoint = torch.load(checkpoint_path, map_location=device)
    if "model_state" not in checkpoint:
        raise RuntimeError(
            f"Checkpoint '{checkpoint_path}' no contiene la llave 'model_state'. "
            "¿Es un checkpoint generado por train.py?"
        )
    model = SmallUNet(in_channels=1, out_channels=1, base_channels=base_channels)
    model.load_state_dict(checkpoint["model_state"])
    model.to(device)
    model.eval()
    val_iou = checkpoint.get("val_iou", None)
    epoch = checkpoint.get("epoch", None)
    return model, val_iou, epoch
