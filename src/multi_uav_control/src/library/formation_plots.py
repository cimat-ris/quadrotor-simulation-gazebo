# encoding: utf-8

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os

def set_axes_radius(ax, origin, radius):
    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])

    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    set_axes_radius(ax, origin, radius)

def set_aspect_equal_3d(ax):
    """Fix equal aspect bug for 3D plots."""

    xlim = ax.get_xlim3d()
    ylim = ax.get_ylim3d()
    zlim = ax.get_zlim3d()

    from numpy import mean
    xmean = mean(xlim)
    ymean = mean(ylim)
    zmean = mean(zlim)

    plot_radius = max([abs(lim - mean_)
                       for lims, mean_ in ((xlim, xmean),
                                           (ylim, ymean),
                                           (zlim, zmean))
                       for lim in lims])

    ax.set_xlim3d([xmean - plot_radius, xmean + plot_radius])
    ax.set_ylim3d([ymean - plot_radius, ymean + plot_radius])
    ax.set_zlim3d([zmean - plot_radius, zmean + plot_radius])

def plot_3d_formation(x_c, obsF=[], path=""):

    col = ['m', '#346ab2', '#fe7f1d', '#00ae36', '#8c544b', 'cyan', 'tomato']

    n = int(x_c.shape[0]/3)
    kmax = x_c.shape[1]

    fig = plt.figure(figsize=(8,8))
    ax = fig.gca(projection='3d')
    for i in range(n):
        #initial
        ax.plot([x_c[i,0], x_c[(i+1)%n,0]], [x_c[n+i,0], x_c[n+(i+1)%n,0]], [x_c[2*n+i,0], x_c[2*n+(i+1)%n,0]], ls='--', color='k', alpha=0.7)
        #trajectory
        ax.plot(x_c[i,:], x_c[i+n,:], x_c[i+2*n,:], color=col[i])
        #final
        ax.plot([x_c[i,kmax-1], x_c[(i+1)%n,kmax-1]], [x_c[n+i,kmax-1], x_c[n+(i+1)%n,kmax-1]], [x_c[2*n+i,kmax-1], x_c[2*n+(i+1)%n,kmax-1]], ls='--', color='red', alpha=0.7)
    #i=0
    #j=2
    #ax.plot([x_c[i,kmax-1], x_c[j,kmax-1]], [x_c[n+i,kmax-1], x_c[n+j,kmax-1]], [x_c[2*n+i,kmax-1], x_c[2*n+j,kmax-1]], ls='--', color='red', alpha=0.7)
    #j=3
    #ax.plot([x_c[i,kmax-1], x_c[j,kmax-1]], [x_c[n+i,kmax-1], x_c[n+j,kmax-1]], [x_c[2*n+i,kmax-1], x_c[2*n+j,kmax-1]], ls='--', color='red', alpha=0.7)
    #i=1
    #j=2
    #ax.plot([x_c[i,kmax-1], x_c[j,kmax-1]], [x_c[n+i,kmax-1], x_c[n+j,kmax-1]], [x_c[2*n+i,kmax-1], x_c[2*n+j,kmax-1]], ls='--', color='red', alpha=0.7)
    #j=3
    #ax.plot([x_c[i,kmax-1], x_c[j,kmax-1]], [x_c[n+i,kmax-1], x_c[n+j,kmax-1]], [x_c[2*n+i,kmax-1], x_c[2*n+j,kmax-1]], ls='--', color='red', alpha=0.7)

    set_axes_equal(ax)
    set_aspect_equal_3d(ax)

    fig.savefig("src/multi_uav_control/output/plots/formation.pdf")
    os.system("pdfcrop src/multi_uav_control/output/plots/formation.pdf src/multi_uav_control/output/plots/formation.pdf")

def plot_2d_formation(x_c, obsF=[], path=""):

    col = ['m', '#346ab2', '#fe7f1d', '#00ae36', '#8c544b', 'cyan', 'tomato']

    n = int(x_c.shape[0]/3)
    kmax = x_c.shape[1]

    fig, ax = plt.subplots(1, 1, figsize=(8,8))
    for i in range(n):
        #initial
        ax.plot([x_c[i,0], x_c[(i+1)%n,0]], [x_c[n+i,0], x_c[n+(i+1)%n,0]], ls='--', color='k', alpha=0.7)
        #trajectory
        ax.plot(x_c[i,:], x_c[i+n,:], color=col[i])
        #final
        ax.plot([x_c[i,kmax-1], x_c[(i+1)%n,kmax-1]], [x_c[n+i,kmax-1], x_c[n+(i+1)%n,kmax-1]], ls='--', color='red', alpha=0.7)
    #i=0
    #j=2
    #ax.plot([x_c[i,kmax-1], x_c[j,kmax-1]], [x_c[n+i,kmax-1], x_c[n+j,kmax-1]], [x_c[2*n+i,kmax-1], x_c[2*n+j,kmax-1]], ls='--', color='red', alpha=0.7)
    #j=3
    #ax.plot([x_c[i,kmax-1], x_c[j,kmax-1]], [x_c[n+i,kmax-1], x_c[n+j,kmax-1]], [x_c[2*n+i,kmax-1], x_c[2*n+j,kmax-1]], ls='--', color='red', alpha=0.7)
    #i=1
    #j=2
    #ax.plot([x_c[i,kmax-1], x_c[j,kmax-1]], [x_c[n+i,kmax-1], x_c[n+j,kmax-1]], [x_c[2*n+i,kmax-1], x_c[2*n+j,kmax-1]], ls='--', color='red', alpha=0.7)
    #j=3
    #ax.plot([x_c[i,kmax-1], x_c[j,kmax-1]], [x_c[n+i,kmax-1], x_c[n+j,kmax-1]], [x_c[2*n+i,kmax-1], x_c[2*n+j,kmax-1]], ls='--', color='red', alpha=0.7)

    ax.set_aspect('equal')
    ax.grid(color='gray', linestyle=':')

    fig.savefig("src/multi_uav_control/output/plots/formation_upper.pdf")
    os.system("pdfcrop src/multi_uav_control/output/plots/formation_upper.pdf src/multi_uav_control/output/plots/formation_upper.pdf")

def plot_3d_virtual(z, obsF=[], path=""):

    col = ['m', '#346ab2', '#fe7f1d', '#00ae36', '#8c544b', 'cyan', 'tomato']

    n = int(z.shape[0]/3)

    fig = plt.figure(figsize=(8,8))
    ax = fig.gca(projection='3d')
    for i in range(n):
        #virtual trajectories
        ax.plot(z[i,:], z[i+n,:], z[i+2*n,:], color=col[i])

    fig.savefig("src/multi_uav_control/output/plots/virtual.pdf")
    os.system("pdfcrop src/multi_uav_control/output/plots/virtual.pdf src/multi_uav_control/output/plots/virtual.pdf")

def plot_controls_u(uList, path="", minu=-10, maxu=10):

    col = ['m', '#346ab2', '#fe7f1d', '#00ae36', '#8c544b', 'cyan', 'tomato']

    fig, axs = plt.subplots(3, 1, figsize=(8,12))
    
    i = 0
    for u in uList:
        kmax = u.shape[1]
        iterations = np.arange(kmax)
        axs[0].plot(iterations, u[0,:], alpha=0.7, color=col[i])
        axs[1].plot(iterations, u[1,:], alpha=0.7, color=col[i])
        axs[2].plot(iterations, u[2,:], alpha=0.7, color=col[i])
        i = i + 1
    
    axs[0].set_ylim(minu-1, maxu+1)
    axs[1].set_ylim(minu-1, maxu+1)
    axs[2].set_ylim(minu-1, maxu+1)
    axs[0].grid(color='gray', linestyle=':')
    axs[0].set_title(r"$u_x$", fontsize=19)
    axs[1].grid(color='gray', linestyle=':')
    axs[1].set_title(r"$u_y$", fontsize=19)
    axs[2].grid(color='gray', linestyle=':')
    axs[2].set_title(r"$u_z$", fontsize=19)

    fig.savefig("src/multi_uav_control/output/plots/inputs.pdf")
    os.system("pdfcrop src/multi_uav_control/output/plots/inputs.pdf src/multi_uav_control/output/plots/inputs.pdf")

def plot_virtual_evolution(z, path=""):

    col = ['m', '#346ab2', '#fe7f1d', '#00ae36', '#8c544b', 'cyan', 'tomato']

    n = int(z.shape[0]/3)
    kmax = z.shape[1]
    iterations = np.arange(kmax)

    fig, axs = plt.subplots(3, 1, figsize=(8,12))

    for i in range(n):
        #trajectory
        axs[0].plot(iterations, z[i,:], alpha=0.7, color=col[i])
        axs[1].plot(iterations, z[i+n,:], alpha=0.7, color=col[i])
        axs[2].plot(iterations, z[i+2*n,:], alpha=0.7, color=col[i])
    
    axs[0].grid(color='gray', linestyle=':')
    axs[0].set_title(r"$x$", fontsize=19)
    axs[1].grid(color='gray', linestyle=':')
    axs[1].set_title(r"$y$", fontsize=19)
    axs[2].grid(color='gray', linestyle=':')
    axs[2].set_title(r"$z$", fontsize=19)

    fig.savefig("src/multi_uav_control/output/plots/virtual_evolution.pdf")
    os.system("pdfcrop src/multi_uav_control/output/plots/virtual_evolution.pdf src/multi_uav_control/output/plots/virtual_evolution.pdf")

def plot_consensus_error(e, path=""):

    col = ['m', '#346ab2', '#fe7f1d', '#00ae36', '#8c544b', 'cyan', 'tomato']

    n = int(e.shape[0]/3)
    kmax = e.shape[1]
    iterations = np.arange(kmax)

    fig, axs = plt.subplots(3, 1, figsize=(8,12))

    axs[0].axhline(color='r', ls='--', lw=2, alpha=0.5)
    axs[1].axhline(color='r', ls='--', lw=2, alpha=0.5)
    axs[2].axhline(color='r', ls='--', lw=2, alpha=0.5)

    for i in range(n):
        #trajectory
        axs[0].plot(iterations, e[i,:], alpha=0.7, color=col[i])
        axs[1].plot(iterations, e[i+n,:], alpha=0.7, color=col[i])
        axs[2].plot(iterations, e[i+2*n,:], alpha=0.7, color=col[i])
    
    axs[0].grid(color='gray', linestyle=':')
    axs[0].set_title(r"$e_x$", fontsize=19)
    axs[1].grid(color='gray', linestyle=':')
    axs[1].set_title(r"$e_y$", fontsize=19)
    axs[2].grid(color='gray', linestyle=':')
    axs[2].set_title(r"$e_z$", fontsize=19)

    fig.savefig("src/multi_uav_control/output/plots/error.pdf")
    os.system("pdfcrop src/multi_uav_control/output/plots/error.pdf src/multi_uav_control/output/plots/error.pdf")

def plot_distances(x, D=0, obsF=[], path="", mind=0.0, maxd=2.5):

    col = ['m', '#346ab2', '#fe7f1d', '#00ae36', '#8c544b', 'cyan', 'tomato']

    n = int(x.shape[0]/3)
    kmax = x.shape[1]
    iterations = np.arange(kmax)

    if obsF:
        fig, ax = plt.subplots(1, 2, figsize=(16, 4))
        for i in range(n):
            xi = np.array([x[i,:], x[i+n,:]]) #2D
            for j in range(i+1,n):
                xj = np.array([x[j,:], x[j+n,:]])
                ax[0].plot(iterations, np.linalg.norm(xi-xj, axis=0), color=col[i], alpha=0.7, lw=2)
                ax[0].plot(iterations, np.linalg.norm(xi-xj, axis=0), '--', color=col[j], alpha=0.7, lw=2)
            for obj in obsF:
                if obj['desc'] == "column":
                    ax[1].plot(iterations, np.linalg.norm(xi - np.array([[obj['x']], [obj['y']]]), axis=0), color=col[i], alpha=0.7, lw=2)
                    #ax[1].plot(iterations, np.linalg.norm(xi - np.array([[obj['x']], [obj['y']]]), axis=0), '--', color='gray', alpha=0.7, lw=2)
        ax[0].set_ylim(mind, maxd)
        ax[0].axhline(D, color='r', lw=2, alpha = 0.7)
        ax[0].grid(color='gray', linestyle='--')
        ax[0].set_title(r"Distance between agents", fontsize=15)
        ax[1].set_ylim(mind, maxd)
        ax[1].axhline(D, color='r', lw=2, alpha = 0.7)
        ax[1].grid(color='gray', linestyle='--')
        ax[1].set_title(u"Distance to obstacle", fontsize=15)
    else:
        fig, ax = plt.subplots(1, 1, figsize=(8, 4))

        for i in range(n):
            xi = np.array([x[i,:], x[i+n,:]])
            for j in range(i+1,n):
                xj = np.array([x[j,:], x[j+n,:]])
                ax.plot(iterations, np.linalg.norm(xi-xj, axis=0), color=col[i], alpha=0.7, lw=2)
                ax.plot(iterations, np.linalg.norm(xi-xj, axis=0), '--', color=col[j], alpha=0.7, lw=2)
        ax.set_ylim(mind, maxd)
        ax.axhline(D, color='r', lw=2, alpha = 0.7)
        ax.grid(color='gray', linestyle=':')
        ax.set_title(r"Distance between agents", fontsize=15)

    fig.savefig("src/multi_uav_control/output/plots/distance.pdf")
    os.system("pdfcrop src/multi_uav_control/output/plots/distance.pdf src/multi_uav_control/output/plots/distance.pdf")

def plot_formaciones_3d(x_con, u_con, d, evasion=False, obsF=[], **kwargs):

    col = ['magenta', 'tab:blue', 'tab:orange', 'tab:green', 'tab:brown']
    
    n = int(x_con.shape[1]/3)
    kmax = len(x_con)
    iteraciones = np.arange(kmax)

    x_r = x_con.copy()
    x_r = x_r - d
    
    #fig1, ax = plt.subplots(1, 2, figsize=(10,5))
    fig1 = plt.figure(figsize=(16,8))
    ax1 = fig1.add_subplot(121, projection='3d')
    ax2 = fig1.add_subplot(122, projection='3d')

    for i in range(n):
        #inicial
        ax1.plot3D([x_r[0,3*i], x_r[0,(3*i+3)%(3*n)]], [x_r[0,3*i+1], x_r[0,(3*i+4)%(3*n)]], [x_r[0,3*i+2], x_r[0,(3*i+5)%(3*n)]], 
                   '--', color='k', alpha=0.7)
        #virtual
        ax2.plot([x_con[0,i*3]], [x_con[0,i*3+1]], [x_con[0,i*3+2]], 
                markerfacecolor='k', markeredgecolor='k', marker='o', markersize=5, alpha=0.6)
        #trayectoria
        ax1.plot3D(x_r[:,3*i], x_r[:,3*i+1], x_r[:,3*i+2], color = col[i])
        #trayectoria virtual
        ax2.plot3D(x_con[:,3*i], x_con[:,3*i+1], x_con[:,3*i+2], color = col[i])
        #final
        ax1.plot3D([x_r[kmax-1,3*i], x_r[kmax-1,(3*i+3)%(3*n)]], 
                   [x_r[kmax-1,3*i+1], x_r[kmax-1,(3*i+4)%(3*n)]], 
                   [x_r[kmax-1,3*i+2], x_r[kmax-1,(3*i+5)%(3*n)]], 
                   '--', color='red', alpha=0.7)
        #virtual
        ax2.plot([x_con[kmax-1,i*3]], [x_con[kmax-1,i*3+1]], [x_con[kmax-1,i*3+2]], 
                markerfacecolor='r', markeredgecolor='r', marker='o', markersize=5, alpha=0.6)
    
    if obsF:
        maxZ = 0
        for i in range(n):
            maxZ = max(maxZ, np.max(x_r[:,3*i+2]))
    for obj in obsF:
        Xc,Yc,Zc = data_for_cylinder_along_z(obj['x'], obj['y'], obj['rad'], 0.0, 1.5*maxZ)
        ax1.plot_surface(Xc, Yc, Zc, alpha=0.5, color='gray')
    
    set_axes_equal(ax1)
    set_aspect_equal_3d(ax1)
    
    fig2, axs = plt.subplots(3, 2, figsize=(16,12))
    
    for i in range(n):
        axs[0,0].plot(iteraciones, x_con[:,3*i], alpha=0.7, color=col[i])
        axs[1,0].plot(iteraciones, x_con[:,3*i+1], alpha=0.7, color=col[i])
        axs[2,0].plot(iteraciones, x_con[:,3*i+2], alpha=0.7, color=col[i])
        axs[0,1].plot(iteraciones, u_con[:,3*i], alpha=0.7, color=col[i])
        axs[1,1].plot(iteraciones, u_con[:,3*i+1], alpha=0.7, color=col[i])
        axs[2,1].plot(iteraciones, u_con[:,3*i+2], alpha=0.7, color=col[i])
        
    axs[0,0].grid(color='gray', linestyle='--')
    axs[0,0].set_title(r"$x$", fontsize=19)
    axs[1,0].grid(color='gray', linestyle='--')
    axs[1,0].set_title(r"$y$", fontsize=19)
    axs[2,0].grid(color='gray', linestyle='--')
    axs[2,0].set_title(r"$z$", fontsize=19)
    axs[0,1].grid(color='gray', linestyle='--')
    axs[0,1].set_title(r"$u_x$", fontsize=19)
    axs[1,1].grid(color='gray', linestyle='--')
    axs[1,1].set_title(r"$u_y$", fontsize=19)
    axs[2,1].grid(color='gray', linestyle='--')
    axs[2,1].set_title(r"$u_z$", fontsize=19)

    if evasion:
        if obsF:
            fig3, ax = plt.subplots(1, 2, figsize=(16, 4))
            for i in range(n):
                xi = x_r[:,3*i:3*(i+1)]
                for j in range(i+1,n):
                    xj = x_r[:,3*j:3*(j+1)]
                    ax[0].plot(iteraciones, np.linalg.norm(xi-xj, axis=1), color=col[i], alpha=0.7, lw=2)
                    ax[0].plot(iteraciones, np.linalg.norm(xi-xj, axis=1), '--', color=col[j], alpha=0.7, lw=2)
                for obj in obsF:
                    if obj['desc'] == "columna":
                        ax[1].plot(iteraciones, np.linalg.norm(xi[:,:2] - np.array([obj['x'], obj['y']]), axis=1), color=col[i], alpha=0.7, lw=2)
                        ax[1].plot(iteraciones, np.linalg.norm(xi[:,:2] - np.array([obj['x'], obj['y']]), axis=1), '--', color='gray', alpha=0.7, lw=2)
            ax[0].axhline(kwargs['D'], color='r', lw=2, alpha = 0.7)
            ax[0].grid(color='gray', linestyle='--')
            ax[0].set_title(r"Distancia entre agentes", fontsize=15)
            ax[1].axhline(kwargs['D'], color='r', lw=2, alpha = 0.7)
            ax[1].grid(color='gray', linestyle='--')
            ax[1].set_title(u"Distancia a los obstáculos", fontsize=15)
        else:
            fig3, ax = plt.subplots(1, 1, figsize=(8, 4))

            for i in range(n):
                xi = x_r[:,3*i:3*(i+1)]
                for j in range(i+1,n):
                    xj = x_r[:,3*j:3*(j+1)]
                    ax.plot(iteraciones, np.linalg.norm(xi-xj, axis=1), color=col[i], alpha=0.7, lw=2)
                    ax.plot(iteraciones, np.linalg.norm(xi-xj, axis=1), '--', color=col[j], alpha=0.7, lw=2)
            ax.axhline(kwargs['D'], color='r', lw=2, alpha = 0.7)
            ax.grid(color='gray', linestyle='--')
            ax.set_title(r"Distancia entre agentes", fontsize=15)