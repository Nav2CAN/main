
from numba import jit
import numpy as np


@jit
def initSocialZones(density, sigmaFront, sigmaSide, sigmaBack, velocities, plotsize=3):
    x = np.arange(-plotsize, plotsize, density)  # [m]
    y = np.arange(-plotsize, plotsize, density)
    zones = []
    for vel in velocities:
        zones.append(makeProxemicZone(
            0, 0, x, y, 0, sigmaFront+(1*vel), sigmaSide, sigmaBack))  # TODO only apply velocity if its negative
    return zones


@jit
def asymetricGaus(x=0, y=0, x0=0, y0=0, theta=0, sigmaFront=2, sigmaSide=4/3, sigmaBack=1) -> float:
    angle: float = np.mod(np.arctan2(y-y0, x-x0), 2*np.pi)+theta
    if (abs(angle) >= np.pi/2 and abs(angle) <= np.pi+np.pi/2):
        sigma: float = sigmaBack
    else:
        sigma: float = sigmaFront

    a: float = ((np.cos(theta) ** 2)/(2*sigma ** 2)) + \
        ((np.sin(theta) ** 2)/(2*sigmaSide ** 2))
    b: float = (np.sin(2*theta)/(4*sigma ** 2)) - \
        (np.sin(2*theta)/(4*sigmaSide ** 2))
    c: float = ((np.sin(theta) ** 2)/(2*sigma ** 2)) + \
        ((np.cos(theta) ** 2)/(2*sigmaSide ** 2))

    return np.exp(-(a*(x-x0) ** 2+2*b*(x-x0)*(y-y0)+c*(y-y0) ** 2))


@jit
def makeProxemicZone(x0, y0, x, y, theta, sigmaFront, sigmaSide, sigmaBack) -> np.ndarray:
    social: np.ndarray = np.zeros((len(x), len(y)), dtype=np.uint8)
    for i in range(len(x)):
        for j in range(len(y)):
            social[j, i] = thresholdCost(asymetricGaus(
                x[i], y[j], x0, y0, theta, sigmaFront))
    return social


@jit
def thresholdCost(cost: float) -> float:
    if cost > asymetricGaus(y=0.5):
        return 255
    if cost > asymetricGaus(y=1.0):
        return np.floor(255*asymetricGaus(y=1.0))
    if cost > asymetricGaus(y=1.5):
        return cost*255
    return 0
