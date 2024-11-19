import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from VisualControlPython.CameraModels.PlanarCamera import PlanarCamera
from VisualControlPython.homography import (homogToRt,H_from_points,Rodrigues,yaw_angle_from_rotationMatrix)

from matplotlib import rc

#rc('text', usetex=True)
# plt.rcParams.update({'font.size': 12})


# Función suavizada de saturación para exponencial decreciente
def sat_smooth_exp(x, L_min):
    return L_min + (1 - L_min) * np.exp(-x)

# Parámetros
x = np.linspace(0, 10, 500)  # Valores de entrada
L_min = 0.26                  # Límite mínimo

# Cálculo de la salida
y = sat_smooth_exp(x, L_min)

# Graficar la función
plt.figure(figsize=(8, 5))
plt.plot(x, y, label=f"sat_smooth_exp(x, L_min={L_min})", color="blue")
plt.axhline(L_min, color="red", linestyle="--", label=f"Límite mínimo ({L_min})")
plt.title("Función Suavizada de Saturación para Exponencial Decreciente")
plt.xlabel("Entrada (x)")
plt.ylabel("Salida (y)")
plt.legend()
plt.grid()
plt.show()




