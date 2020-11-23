import numpy as np
import matplotlib.pyplot as plt

knownPtsCadence    = [100, 100, 100, 100, 100,  80,  80,  80,  80,  80]
knownPtsResistance = [ 30,  35,  40,  45,  50,  30,  35,  40,  45,  50]
knownPtsPowerMin   = [ 88, 120, 160, 215, 260,  58,  83, 111, 143, 186]
knownPtsPowerMax   = [ 92, 125, 165, 220, 265,  62,  85, 115, 146, 190]

plt.scatter(knownPtsPowerMin, knownPtsResistance)
plt.scatter(knownPtsPowerMax, knownPtsResistance)

xlist = np.arange(0,300)
for cadence in range (40,111,10):
    # Option 1: https://www.reddit.com/r/pelotoncycle/comments/gwpyfw/diy_peloton_resistance_output/
    # $Resistance = (145*($Power/(11.29*  ($Cadence-22.5)^1.25)  )^(.4651))
    ylist = (145*pow(xlist/(11.29*pow(max((cadence-22.5),1),1.25)),(.4651)))
    # Option 2: https://www.reddit.com/r/pelotoncycle/comments/b0bulz/how_the_peloton_bike_calculates_output_and_speed/
    # Resistance = 100 * Output / (2.5*24*(cadence-35))
    zlist = 100 * xlist / (2.5*24*(cadence-35))
    plt.plot(xlist,ylist, label='%drpm' % (cadence))
    #plt.plot(xlist,zlist, label='%drpm option 2' % (cadence))
plt.xlabel("Power")
plt.ylabel("Resistance")
plt.title('Bike Calibration')
plt.legend()
plt.show()
