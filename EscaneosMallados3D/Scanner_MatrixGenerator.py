# Importamos librerías

import numpy as np


def normaliza(v, tol=1e-6):
    """
    Devuelve, sin alterarlo, el vector normalizado de v.

    Parameters
    ----------
    v : list, tuple, numpy.ndarray: numérico
        Vector unidimensional
    tol : float
        Tolerancia que debe cumplir la norma del vector v
    Returns
    -------
    v_norm : numpy.ndarray of floats
        Vector normalizado unitario
    Raises
    ------
    ValueError
        Si la norma del vector inicial es inferior a tol
    Example
    -------
        >>> v = (1, 1, 1)
        >>> normaliza(v)
    """

    norma = np.linalg.norm(v)
    if norma < tol:
        raise ValueError(
            "La norma del vector es inferior a la tolerancia {}.".format(tol)
        )
    v_norm = v / norma  # Al usar np.linalg.norm() v_norm es un numpy.ndarray
    return v_norm


v = (1, 1, 1)
normaliza(v)


def distancia_entre_puntos(p1, p2):  # Versión solo válida para numpy arrays
    """
    Parameters
    ----------
    p1, p2 : numpy.ndarray
        Vectores unidimensionales
    Returns
    -------
    dist : numpy float
        Distancia entre los puntos p1 y p2
    Example:
    --------
    >>> p1 = np.array((1, 1, 1), dtype=float)
    >>> p2 = np.array((2, 3, 4), dtype=float)
    >>> distancia_entre_puntos(p1, p2)
    """
    dist = np.linalg.norm(p1 - p2)  # p1-p2 no está definido para tuplas o listas
    return dist


def angulo(v1, v2):
    """
    Parameters
    ----------
    v1, v2 : tuple, list, numpy.ndarray: numérico
        Vectores unidimensionales
    Returns
    -------
    angulo : float
        Ángulo en radianes que forman los vectores
    Example
    -------
    >>> u_x = [1, 0, 0]  # Vector unitario eje X
    >>> u_y = [0, 1, 0]  # Vector unitario eje Y
    >>> angulo(u_x, u_y)*180/np.pi  # Ángulo en grados
    """

    prod_esc = np.dot(v1, v2)
    norma_v1 = np.linalg.norm(v1)
    norma_v2 = np.linalg.norm(v2)

    angulo = np.arccos(np.clip(prod_esc / (norma_v1 * norma_v2), -1, 1))
    return angulo


u_x = [1, 0, 0]  # Vector unitario eje X
u_y = [0, 1, 0]  # Vector unitario eje Y


ang = angulo(u_x, u_y)
print(ang * 180 / np.pi)


def plano_tres_puntos(p1, p2, p3, tol=1e-3):
    """
    Parameters
    ----------
    p1, p2, p3 : numpy.ndarray of floats
        Vectores unidimensionales representando 3 puntos 3D
    tol : float
        Tolerancia que deben cumplir las distancias entre los puntos
    Returns
    -------
    n, D : tuple : (numpy.ndarray of float, numpy float)
        n es el vector unitario perpendicular al plano
        D es el término independiente de la ecuación algebraica Ax+By+Cz+D=0
    Raises
    ------
    ValueError
        Si la distancia entre alguno de los 3 puntos es inferior a tol
    Example
    -------
    >>> p1 = np.array((1, 0, 0))
    >>> p2 = np.array((0, 1, 0))
    >>> p3 = np.array((0, 0, 1))
    >>> n, D = plano_tres_puntos(p1, p2, p3)
    """
    dist12 = distancia_entre_puntos(p1, p2)
    dist13 = distancia_entre_puntos(p1, p3)
    dist23 = distancia_entre_puntos(p2, p3)

    if dist12 < tol or dist13 < tol or dist23 < tol:
        raise ValueError(
            "Distancia entre puntos menor que la tolerancia {}.".format(tol)
        )

    # Producto vectorial para A B C
    # |  i       j       k    |
    # |v_12[0] v_12[1] v_12[2]|
    # |v_13[0] v_13[1] v_13[2]|

    v12 = p1 - p2
    v13 = p1 - p3

    n = normaliza(np.cross(v12, v13))

    D = -np.dot(n, p1)

    return n, D


def recta_dos_puntos(p1, p2, tol=1e-3):
    """
    Parameters
    ----------
    p1, p2 : numpy.ndarray of floats
        Vectores unidimensionales representando 2 puntos 3D
    tol : float
        Tolerancia que debe cumplir la distancia entre los puntos
    Returns
    -------
    p, v : tuple: (numpy.ndarray of float, numpy.ndarray of float)
        p es una copia de p1, punto pasado como primer parámetro

        v es el vector director unitario de la recta
    Raises
    ------
    ValueError
        Si la distancia entre los 2 puntos es inferior a tol
    Example
    -------
    >>> p1 = np.array((2, 2, 2))
    >>> p2 = np.array((3, 3, 3))
    >>> p, v = recta_dos_puntos(p1, p2)
    """
    if distancia_entre_puntos(p1, p2) < tol:
        raise ValueError(
            "Distancia entre puntos menor que la tolerancia {}.".format(tol)
        )
    v = normaliza(p1 - p2)
    return np.copy(p1), v


def interseccion_recta_plano(recta, plano, tol=0.1):
    """
    Parameters
    ----------
    recta : tuple : (numpy.ndarray of floats, numpy.ndarray of floats)
        Recta definida por la tupla punto inicial y vector director
        p=p_0+l*v
    plano : tuple : (numpy.ndarray of floats, float)
        Plano definido por la tupla vector normal (A, B, C) y término independiente D
        Ax+By+Cz+D=0
    tol : float
        Tolerancia que deben cumplir el ángulo entre recta y plano
    Returns
    -------
    p : numpy.ndarray of floats
        El punto de intersección
    Raises
    ------
    ValueError
        Si el ángulo entre ambas figuras es inferior a tol en radianes
    Example
    -------
    >>> p1 = np.array((1, 0, 0))
    >>> p2 = np.array((0, 1, 0))
    >>> p3 = np.array((0, 0, 1))
    >>> p4 = np.array((2, 2, 2))
    >>> p5 = np.array((3, 3, 3))
    >>> plano = plano_tres_puntos(p1, p2, p3)
    >>> recta = recta_dos_puntos(p4, p5)
    >>> p = interseccion_recta_plano(recta, plano)
    """
    ang = angulo(recta[1], plano[0])

    if np.abs(ang - np.pi / 2) < tol:
        raise ValueError(
            "La recta y el plano son casi paralelos según tolerancia {}.".format(tol)
        )

    lamb = -(np.dot(recta[0], plano[0]) + plano[1]) / np.dot(recta[1], plano[0])

    return recta[0] + lamb * recta[1]


# Ejemplo interseccion_recta_plano()
p1 = np.array((0, 0, 0))
p2 = np.array((0, 0, 1))
# Para plano izquierdo
p3 = np.array((1, np.tan(30 * np.pi / 180), 0))
# Para plano derecho
# p3 = np.array((-1, np.tan(30 * np.pi / 180), 0))

p4 = np.array((0, 300, 0))
p5 = np.array((1, 304.1, 1))

plano = plano_tres_puntos(p1, p2, p3)
recta = recta_dos_puntos(p4, p5)

p = interseccion_recta_plano(recta, plano)

mtx_pos_x = []
mtx_pos_y = []
mtx_pos_z = []

aux = []
dx = 5.33 / 1920
dz = 4 / 1080

# Medidas obtenidas de https://www.ienso.com/product/ism-imx307/

dx = 0.0029
dz = 0.0029

punto_o = np.array((0, 310, 0))

for fila in range(int(1080 / 2), int(-1080 / 2) - 1, -1):
    if fila != 0:
        aux_x = []
        aux_y = []
        aux_z = []
        for col in range(int(-1920 / 2), int(1920 / 2) + 1):
            if col != 0:
                recta = recta_dos_puntos(
                    punto_o, np.array((col * dz, 314.1, fila * dx))
                )
                aux_x.append(
                    np.around(interseccion_recta_plano(recta, plano), 7).tolist()[0]
                )
                aux_y.append(
                    np.around(interseccion_recta_plano(recta, plano), 7).tolist()[1]
                )
                aux_z.append(
                    np.around(interseccion_recta_plano(recta, plano), 7).tolist()[2]
                )
        mtx_pos_x.append(aux_x)
        mtx_pos_y.append(aux_y)
        mtx_pos_z.append(aux_z)


np.savetxt("mtx_x_izq.txt", mtx_pos_x)
np.savetxt("mtx_y_izq.txt", mtx_pos_y)
np.savetxt("mtx_z_izq.txt", mtx_pos_z)
