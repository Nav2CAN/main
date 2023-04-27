import numpy as np
import matplotlib.pyplot as plt
from numba import jit


@jit(nopython=True)
def asymetricGaus(x=0, y=0, x0=0, y0=0, theta=0, sigmaFront=2, sigmaSide=4/3, sigmaBack=1) -> float:
    angle = np.mod(np.arctan2(y-y0, x-x0), 2*np.pi)+theta
    if (abs(angle) >= np.pi/2 and abs(angle) <= np.pi+np.pi/2):
        sigma = sigmaBack
    else:
        sigma = sigmaFront

    a = ((np.cos(theta) ** 2)/(2*sigma ** 2)) + \
        ((np.sin(theta) ** 2)/(2*sigmaSide ** 2))
    b = (np.sin(2*theta)/(4*sigma ** 2)) - (np.sin(2*theta)/(4*sigmaSide ** 2))
    c = ((np.sin(theta) ** 2)/(2*sigma ** 2)) + \
        ((np.cos(theta) ** 2)/(2*sigmaSide ** 2))

    return np.exp(-(a*(x-x0) ** 2+2*b*(x-x0)*(y-y0)+c*(y-y0) ** 2))


@jit(nopython=True)
def makeProxemicZone(x0, y0, x, y, theta, sigmaFront, sigmaSide, sigmaBack) -> np.ndarray:
    social = np.zeros((len(x), len(y)))
    for i in range(len(x)):
        for j in range(len(y)):
            social[j, i] = thresholdCost(asymetricGaus(
                x[i], y[j], x0, y0, theta, sigmaFront))
            # passing[j, i] = asymetricGaus(
            #     x[i], y[j], x0, y0, theta-np.pi/2, 2, 2/3, sigmaBack)

    # social=max(social,passing);
    return social


@jit(nopython=True)
def thresholdCost(cost: float) -> float:
    if cost > asymetricGaus(y=0.5):
        return 255
    if cost > asymetricGaus(y=1.0):
        return 255*asymetricGaus(y=1.0)
    if cost > asymetricGaus(y=1.5):
        return cost*255
    return 0


@jit
def main():
    x0 = 0
    y0 = 0
    theta = 0
    # %standard diviations %adjust to get different shapes
    sigmaFront = 2
    sigmaSide = 4/3
    sigmaBack = 1

    # %plot size and res
    plotsize = 3
    density = 0.05
    resolution = round(2*plotsize/density)

    # %create canvas
    x = np.linspace(x0-plotsize+1, x0+plotsize+1, resolution)
    y = np.linspace(y0-plotsize, y0+plotsize, resolution)
    # calcthreshold(0, 2, x, y, np.pi+np.pi/2, sigmaFront, sigmaSide, sigmaBack)
    velocities = np.arange(0, 1.5, 0.1)
    socialZones = []
    for vel in velocities:
        zone = makeProxemicZone(
            0, 0, x, y, np.pi/2, sigmaFront+vel, sigmaSide, sigmaBack)
        socialZones.append(zone)
        plt.figure()
        plt.imshow(zone)
    plt.show()


if __name__ == "__main__":
    main()
