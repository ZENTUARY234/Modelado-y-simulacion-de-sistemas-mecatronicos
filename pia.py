Python 3.13.15 (tags/v3.13.15:4061bc4, Aug  5 2026, 13:05:39) [MSC v.1944 64 bit (AMD64)] on win32
Enter "help" below or click "Help" above for more information.

# PIA - SIMULACION DE ROBOT ESFERICO DE 3 GDL


import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# PARAMETROS DEL ROBOT
m1 = 1.0
m2 = 2.0
l1 = 1.0
g = 9.81

# ENTRADAS CONSTANTES
# tau1 y tau2 en Nm; tau3 en N
def par(t):
    return np.array([0.02, 5.55, -6.94])


# MODELO DINAMICO
def modelo_robot(x, t, m2, g, par):

    q1, q2, q3, qd1, qd2, qd3 = x

    qd = np.array([qd1, qd2, qd3])

    # Matriz de inercia M(q)
    M11 = (m2 / 4) * q3**2 * np.sin(q2)**2
    M22 = (m2 / 4) * q3**2
    M33 = m2 / 4

    Mq = np.array([
        [M11, 0.0, 0.0],
        [0.0, M22, 0.0],
        [0.0, 0.0, M33]
    ])
... 
...     # Matriz de Coriolis C(q,qd)
...     C11 = ((m2/2)*q3*qd3*np.sin(q2)**2
...            + (m2/2)*q3**2*np.sin(q2)
...            * np.cos(q2)*qd2)
... 
...     C21 = -(m2/4)*q3**2*np.sin(q2)*np.cos(q2)*qd1
...     C22 = (m2/2)*q3*qd3
... 
...     C31 = -(m2/4)*q3*np.sin(q2)**2*qd1
...     C32 = -(m2/4)*q3*qd2
... 
...     Cq = np.array([
...         [C11, 0.0, 0.0],
...         [C21, C22, 0.0],
...         [C31, C32, 0.0]
...     ])
... 
...     # Vector de gravedad G(q)
...     G1 = 0.0
...     G2 = (m2*g*q3/2)*np.sin(q2)
...     G3 = -(m2*g/2)*np.cos(q2)
... 
...     Gq = np.array([G1, G2, G3])
... 
...     # Entradas aplicadas
...     tau = par(t)
... 
...     # Calcular las aceleraciones
...     qdd = np.linalg.solve(Mq, tau - Cq @ qd - Gq)
... 
...     return [qd1, qd2, qd3,
...             qdd[0], qdd[1], qdd[2]]
... 
... 
... # TIEMPO DE SIMULACION
... t = np.linspace(0, 1.5, 600)
... 
... 
... # CASO A
x0_a = [np.radians(20), np.radians(45),
        0.80, 0.0, 0.0, 0.0]

sol_a = odeint(
    modelo_robot, x0_a, t,
    args=(m2, g, par)
)


# CASO B
x0_b = [np.radians(40), np.radians(43),
        0.80, 0.0, 0.0, 0.0]

sol_b = odeint(
    modelo_robot, x0_b, t,
    args=(m2, g, par)
)


# RESULTADOS DE LOS DOS CASOS
q1_a, q2_a, q3_a = sol_a[:, 0], sol_a[:, 1], sol_a[:, 2]
qd1_a, qd2_a, qd3_a = sol_a[:, 3], sol_a[:, 4], sol_a[:, 5]

q1_b, q2_b, q3_b = sol_b[:, 0], sol_b[:, 1], sol_b[:, 2]
qd1_b, qd2_b, qd3_b = sol_b[:, 3], sol_b[:, 4], sol_b[:, 5]


# GRAFICAS DEL CASO A
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

ax1.plot(t, np.degrees(q1_a), label="q1")
ax1.plot(t, np.degrees(q2_a), label="q2")
ax1.set_title("Posiciones angulares")
ax1.set_xlabel("Tiempo [s]")
ax1.set_ylabel("Angulo [grados]")
ax1.grid()
ax1.legend()

ax2.plot(t, q3_a, label="q3")
ax2.set_title("Posicion prismatica")
ax2.set_xlabel("Tiempo [s]")
ax2.set_ylabel("Extension [m]")
ax2.grid()
ax2.legend()

fig.suptitle("Caso A - Posiciones")
plt.tight_layout()
plt.savefig("PIA_Caso_A_Posiciones.png", dpi=160)
plt.show()


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

ax1.plot(t, qd1_a, label="dq1/dt")
ax1.plot(t, qd2_a, label="dq2/dt")
ax1.set_title("Velocidades angulares")
ax1.set_xlabel("Tiempo [s]")
ax1.set_ylabel("Velocidad [rad/s]")
ax1.grid()
ax1.legend()

ax2.plot(t, qd3_a, label="dq3/dt")
ax2.set_title("Velocidad prismatica")
ax2.set_xlabel("Tiempo [s]")
ax2.set_ylabel("Velocidad [m/s]")
ax2.grid()
ax2.legend()

fig.suptitle("Caso A - Velocidades")
plt.tight_layout()
plt.savefig("PIA_Caso_A_Velocidades.png", dpi=160)
plt.show()


# GRAFICAS DEL CASO B
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

ax1.plot(t, np.degrees(q1_b), label="q1")
ax1.plot(t, np.degrees(q2_b), label="q2")
ax1.set_title("Posiciones angulares")
ax1.set_xlabel("Tiempo [s]")
ax1.set_ylabel("Angulo [grados]")
ax1.grid()
ax1.legend()

ax2.plot(t, q3_b, label="q3")
ax2.set_title("Posicion prismatica")
ax2.set_xlabel("Tiempo [s]")
ax2.set_ylabel("Extension [m]")
ax2.grid()
ax2.legend()

fig.suptitle("Caso B - Posiciones")
plt.tight_layout()
plt.savefig("PIA_Caso_B_Posiciones.png", dpi=160)
plt.show()


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

ax1.plot(t, qd1_b, label="dq1/dt")
ax1.plot(t, qd2_b, label="dq2/dt")
ax1.set_title("Velocidades angulares")
ax1.set_xlabel("Tiempo [s]")
ax1.set_ylabel("Velocidad [rad/s]")
ax1.grid()
ax1.legend()

ax2.plot(t, qd3_b, label="dq3/dt")
ax2.set_title("Velocidad prismatica")
ax2.set_xlabel("Tiempo [s]")
ax2.set_ylabel("Velocidad [m/s]")
ax2.grid()
ax2.legend()

fig.suptitle("Caso B - Velocidades")
plt.tight_layout()
plt.savefig("PIA_Caso_B_Velocidades.png", dpi=160)
plt.show()

print("Simulaciones terminadas correctamente")
