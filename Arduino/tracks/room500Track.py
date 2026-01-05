from trackBuilder import Track
from numpy import *
from matplotlib.pyplot import *

#define our track based on a list of length values and a list of angle values.
#for a straight, the entry in lengths means how long the straight is, and
#the corresponding angles entry should be 0. For a turn, the length value
#is the turn radius, and the angle value is the subtended angle of the turn.

lengths = [2,1.5,2,1.5];
angles = [0,pi,0,pi];

#now use the trackbuilder's Track class to define the track object.
track = Track(lengths, angles, dS = 0.02,x0=2,y0=1.5)

maptoolsFileName = "./room500track.csv"
maptoolsString = ""
for k in range(0,len(track.S)-1):
    #append with this point
    maptoolsString+= str(track.X[k])+","+str(track.Y[k])+","+str(0)+"\r\n"


#overwrite maptools file with track data.
with open(maptoolsFileName,'w',encoding='utf-8') as file:
    file.write(maptoolsString)
file.close()


#now, plot the track to make sure it looks right.
figure()
plot(track.X,track.Y,'k')
xlabel('X (m)')
ylabel('Y (m)')
axis('equal')
show()
