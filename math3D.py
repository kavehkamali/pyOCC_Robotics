import numpy as np

def AngleAxis(angle,axis):
    x = axis[0, 0]
    y = axis[1, 0]
    z = axis[2, 0]

    c=np.cos(angle)
    s=np.sin(angle)
    v=1-c

    R=np.matrix([[x*x*v+c,   x*y*v-z*s, x*z*v+y*s],
                 [x*y*v+z*s, y*y*v+c,   y*z*v-x*s],
                 [x*z*v-y*s, y*z*v+x*s, z*z*v+c  ]])
    return R
