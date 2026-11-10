Python 3.13.15 (tags/v3.13.15:4061bc4, Aug  5 2026, 13:05:39) [MSC v.1944 64 bit (AMD64)] on win32
Enter "help" below or click "Help" above for more information.

# ACTIVIDAD FUNDAMENTAL 6
# Simulacion de manipulador robotico de dos eslabones


# IMPORTACION DE BIBLIOTECAS
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

print("Librerias listas")

# 1. PARAMETROS DEL MANIPULADOR


# Valores de ejemplo

m1 = 1.0       # Masa del eslabon 1 [kg]
m2 = 1.0       # Masa del eslabon 2 [kg]

l1 = 0.5       # Longitud del eslabon 1 [m]
l2 = 0.4       # Longitud del eslabon 2 [m]

lc1 = l1 / 2   # Centro de gravedad del eslabon 1
lc2 = l2 / 2   # Centro de gravedad del eslabon 2

# Inercias 
I1 = m1 * l1**2 / 12
I2 = m2 * l2**2 / 12

g = 9.81      



# 2. TORQUES APLICADOS


# CASO A
# Se suponen torques nulos
def par_a(t):
    return np.array([0.0, 0.0])


# CASO B
# Torques establecidos en el ejercicio
def par_b(t):
    return np.array([10.0, 1.0])



# 3. MODELO DINAMICO DEL MANIPULADOR


def modelo_manipulador(x, t, m1, m2, l1, l2,
                       lc1, lc2, I1, I2, g, par):

    # Variables de estado
    q1, q2, qd1, qd2 = x

    # Velocidades angulares
    qd = np.array([qd1, qd2])

    
    # MATRIZ DE INERCIA M(q)
    

    M11 = (m1*lc1**2
           + m2*(l1**2 + lc2**2
           + 2*l1*lc2*np.cos(q2))
           + I1 + I2)

    M12 = (m2*(lc2**2
           + l1*lc2*np.cos(q2)) + I2)

    M22 = m2*lc2**2 + I2

    Mq = np.array([[M11, M12],
                   [M12, M22]])

    
    # MATRIZ DE CORIOLIS C(q, qd)
    

    h = m2*l1*lc2*np.sin(q2)

    Cq = np.array([
        [-h*qd2, -h*(qd1 + qd2)],
        [h*qd1, 0.0]
    ])

    
    # VECTOR DE GRAVEDAD G(q)
    

    G1 = ((m1*lc1 + m2*l1)*g*np.sin(q1)
          + m2*g*lc2*np.sin(q1 + q2))

    G2 = m2*g*lc2*np.sin(q1 + q2)

    Gq = np.array([G1, G2])

    
    # TORQUES Y ACELERACIONES
   

    tau = par(t)

    # Ecuacion dinamica:
    # M(q)*qdd + C(q,qd)*qd + G(q) = tau

    qdd = np.linalg.solve(
        Mq, tau - Cq @ qd - Gq
    )

    # Derivadas del sistema
    return [qd1, qd2, qdd[0], qdd[1]]



# 4. TIEMPO DE SIMULACION


t = np.linspace(0, 3, 600)



# 5. SIMULACION DEL CASO A


# Condiciones iniciales:
# q1 = 60 grados
# q2 = 0 grados
# Velocidades iniciales nulas (supuesto)

x0_a = [np.radians(60), np.radians(0), 0.0, 0.0]

sol_a = odeint(
    modelo_manipulador, x0_a, t,
    args=(m1, m2, l1, l2, lc1, lc2,
          I1, I2, g, par_a)
)

# Resultados del caso A
q1_a = sol_a[:, 0]
q2_a = sol_a[:, 1]

qd1_a = sol_a[:, 2]
qd2_a = sol_a[:, 3]



# 6. SIMULACION DEL CASO B


# Condiciones iniciales:
# q1 = 0 grados
# q2 = 0 grados
# Velocidades iniciales nulas (supuesto)
# Torque 1 = 10 Nm
# Torque 2 = 1 Nm

x0_b = [np.radians(0), np.radians(0), 0.0, 0.0]

sol_b = odeint(
    modelo_manipulador, x0_b, t,
    args=(m1, m2, l1, l2, lc1, lc2,
          I1, I2, g, par_b)
)

# Resultados del caso B
q1_b = sol_b[:, 0]
q2_b = sol_b[:, 1]

qd1_b = sol_b[:, 2]
qd2_b = sol_b[:, 3]



# 7. GRAFICAS DEL CASO A


fig, (ax1, ax2) = plt.subplots(
    1, 2, figsize=(12, 4)
)

# Posiciones angulares
ax1.plot(t, np.degrees(q1_a),
         label="q1", linewidth=2)

ax1.plot(t, np.degrees(q2_a),
         label="q2", linewidth=2)

ax1.set_title("Posiciones angulares")
ax1.set_xlabel("Tiempo [s]")
ax1.set_ylabel("Angulo [grados]")
ax1.grid()
ax1.legend()

# Velocidades angulares
ax2.plot(t, qd1_a,
         label="Velocidad q1", linewidth=2)

ax2.plot(t, qd2_a,
         label="Velocidad q2", linewidth=2)

ax2.set_title("Velocidades angulares")
ax2.set_xlabel("Tiempo [s]")
ax2.set_ylabel("Velocidad [rad/s]")
ax2.grid()
ax2.legend()

fig.suptitle("CASO A - Condicion inicial de 60 grados")

plt.tight_layout()

# Guardar grafica
plt.savefig("AF6_Caso_A.png", dpi=160)

plt.show()


# =============================================
# 8. GRAFICAS DEL CASO B
# =============================================

fig, (ax1, ax2) = plt.subplots(
...     1, 2, figsize=(12, 4)
... )
... 
... # Posiciones angulares
... ax1.plot(t, np.degrees(q1_b),
...          label="q1", linewidth=2)
... 
... ax1.plot(t, np.degrees(q2_b),
...          label="q2", linewidth=2)
... 
... ax1.set_title("Posiciones angulares")
... ax1.set_xlabel("Tiempo [s]")
... ax1.set_ylabel("Angulo [grados]")
... ax1.grid()
... ax1.legend()
... 
... # Velocidades angulares
... ax2.plot(t, qd1_b,
...          label="Velocidad q1", linewidth=2)
... 
... ax2.plot(t, qd2_b,
...          label="Velocidad q2", linewidth=2)
... 
... ax2.set_title("Velocidades angulares")
... ax2.set_xlabel("Tiempo [s]")
... ax2.set_ylabel("Velocidad [rad/s]")
... ax2.grid()
... ax2.legend()
... 
... fig.suptitle("CASO B - Torques de 10 y 1 Nm")
... 
... plt.tight_layout()
... 
... # Guardar grafica
... plt.savefig("AF6_Caso_B.png", dpi=160)
... 
... plt.show()
... 
... print("Simulaciones terminadas correctamente")
