#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt


def r2(angle,a,b,k):
    return (b*np.cos(angle)+a*k*np.sin(angle))**2+a**2*np.sin(angle)**2

if __name__ == "__main__":
    a = 1

    b_arr = np.linspace(0.5, 2, num=200)
    k_arr = np.linspace(-1, 1, num=201)

    orig_elong = b_arr/a

    orig_elong[orig_elong<1.] = 1./orig_elong[orig_elong<1.]


    ks,bs = np.meshgrid(k_arr, b_arr)

    res = np.zeros_like(ks)
    

    angles = np.linspace(0,2.*np.pi,720)

    for i_b in range(len(b_arr)):
        b = b_arr[i_b]

        for i_k in range(len(k_arr)):
            k = k_arr[i_k]

            xs = b*np.cos(angles) + a*k*np.sin(angles)
            ys = a*np.sin(angles)

            lens = r2(angles,a,b,k)

            max_angle_before = angles[lens.argmax()]
            min_angle_before = angles[lens.argmin()]

            # calc_angle = np.arctan(-2*a*b*k/(b**2 * (k**2+1) -a**2))/2.
            
            # res[i_k, i_b] = np.sqrt(r2(calc_angle,a,b,k)/r2(calc_angle +np.pi/2.,a,b,k))-orig_elong[i_b]
            res[i_b, i_k] = np.sqrt(lens.max()/lens.min())/orig_elong[i_b]
            #print(k, b, ":",np.sqrt(lens.max()), "/", np.sqrt(lens.min()), "-",orig_elong[i_b] ,"->", res[i_b,i_k])

    plt.clf()
    plt.pcolor(ks, bs, res,cmap="inferno")
    plt.xlabel("k")
    plt.ylabel("b")
    plt.colorbar()
    plt.show()