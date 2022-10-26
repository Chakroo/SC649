#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt


n=64
pitch=0.1
t=np.linspace(0,((n-1)*pitch),n)
A=4
a=2
b=1
xp=A*np.cos(a*t)
yp=A*np.sin(b*t)
out = np.c_[t,xp,yp]
print (out)
plt.plot(xp,yp)
plt.xlabel('xp')
plt.ylabel('yp')
plt.savefig('waypoints.png')
plt.show()
mat=np.matrix(out)

with open('waypoints.txt','w+') as f:
    f.write("t xp yp\n")
    for line in mat:
        np.savetxt(f,line,fmt='%.3f')
f.close()