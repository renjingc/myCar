# -*- coding: utf-8 -*-

"""Copyright 2015 Roger R Labbe Jr.

FilterPy library.
http://github.com/rlabbe/filterpy

Documentation at:
https://filterpy.readthedocs.org

Supporting book at:
https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python

This is licensed under an MIT license. See the readme.MD file
for more information.
6"""

from __future__ import (absolute_import, division, print_function,
                        unicode_literals)


import matplotlib.pyplot as plt
import numpy.random as random
from numpy.random import randn
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import (unscented_transform, MerweScaledSigmaPoints,
                             JulierSigmaPoints)
from filterpy.common import Q_discrete_white_noise
import filterpy.stats as stats
from math import cos, sin


DO_PLOT = False


def test_sigma_plot():
    """ Test to make sure sigma's correctly mirror the shape and orientation
    of the covariance array."""

    x = np.array([[1, 2]])
    P = np.array([[2, 1.2],
                  [1.2, 2]])
    kappa = .1

    # if kappa is larger, than points shoudld be closer together

    sp0 = JulierSigmaPoints(n=2, kappa=kappa)
    sp1 = JulierSigmaPoints(n=2, kappa=kappa*1000)

    w0, _ = sp0.weights()
    w1, _ = sp1.weights()

    Xi0 = sp0.sigma_points (x, P)
    Xi1 = sp1.sigma_points (x, P)

    assert max(Xi1[:,0]) > max(Xi0[:,0])
    assert max(Xi1[:,1]) > max(Xi0[:,1])

    if DO_PLOT:
        plt.figure()
        for i in range(Xi0.shape[0]):
            plt.scatter((Xi0[i,0]-x[0, 0])*w0[i] + x[0, 0],
                        (Xi0[i,1]-x[0, 1])*w0[i] + x[0, 1],
                         color='blue')

        for i in range(Xi1.shape[0]):
            plt.scatter((Xi1[i, 0]-x[0, 0]) * w1[i] + x[0,0],
                        (Xi1[i, 1]-x[0, 1]) * w1[i] + x[0,1],
                         color='green')

        stats.plot_covariance_ellipse([1, 2], P)


def test_julier_weights():
    for n in range(1,15):
        for k in np.linspace(0,5,0.1):
            Wm = UKF.weights(n, k)

            assert abs(sum(Wm) - 1) < 1.e-12


def test_scaled_weights():
    for n in range(1,5):
        for alpha in np.linspace(0.99, 1.01, 100):
            for beta in range(0,2):
                for kappa in range(0,2):
                    sp = MerweScaledSigmaPoints(n, alpha, 0, 3-n)
                    Wm, Wc = sp.weights()
                    assert abs(sum(Wm) - 1) < 1.e-1
                    assert abs(sum(Wc) - 1) < 1.e-1


def test_sigma_points_1D():
    """ tests passing 1D data into sigma_points"""

    kappa = 0.
    sp = JulierSigmaPoints(1, kappa)

    #ukf = UKF(dim_x=1, dim_z=1, dt=0.1, hx=None, fx=None, kappa=kappa)

    Wm, Wc = sp.weights()
    assert np.allclose(Wm, Wc, 1e-12)
    assert len(Wm) == 3

    mean = 5
    cov = 9

    Xi = sp.sigma_points (mean, cov)
    xm, ucov = unscented_transform(Xi,Wm, Wc, 0)

    # sum of weights*sigma points should be the original mean
    m = 0.0
    for x, w in zip(Xi, Wm):
        m += x*w

    assert abs(m-mean) < 1.e-12
    assert abs(xm[0] - mean) < 1.e-12
    assert abs(ucov[0,0]-cov) < 1.e-12

    assert Xi.shape == (3,1)



class RadarSim(object):
    def __init__(self, dt):
        self.x = 0
        self.dt = dt

    def get_range(self):
        vel = 100  + 5*randn()
        alt = 1000 + 10*randn()
        self.x += vel*self.dt

        v = self.x * 0.05*randn()
        rng = (self.x**2 + alt**2)**.5 + v
        return rng


def test_radar():
    def fx(x, dt):
        A = np.eye(3) + dt * np.array ([[0, 1, 0],
                                        [0, 0, 0],
                                        [0, 0, 0]])
        return A.dot(x)

    def hx(x):
        return np.sqrt (x[0]**2 + x[2]**2)

    dt = 0.05

    sp = JulierSigmaPoints(n=3, kappa=0.)
    kf = UKF(3, 1, dt, fx=fx, hx=hx, points=sp)

    kf.Q *= 0.01
    kf.R = 10
    kf.x = np.array([0., 90., 1100.])
    kf.P *= 100.
    radar = RadarSim(dt)

    t = np.arange(0,20+dt, dt)

    n = len(t)

    xs = np.zeros((n,3))

    random.seed(200)
    rs = []
    #xs = []
    for i in range(len(t)):
        r = radar.get_range()
        #r = GetRadar(dt)
        kf.predict()
        kf.update(z=[r])

        xs[i,:] = kf.x
        rs.append(r)

    if DO_PLOT:
        print(xs[:,0].shape)

        plt.figure()
        plt.subplot(311)
        plt.plot(t, xs[:,0])
        plt.subplot(312)
        plt.plot(t, xs[:,1])
        plt.subplot(313)

        plt.plot(t, xs[:,2])


def test_linear_2d():
    """ should work like a linear KF if problem is linear """


    def fx(x, dt):
        F = np.array([[1, dt, 0, 0],
                      [0,  1, 0, 0],
                      [0, 0,  1, dt],
                      [0, 0, 0,  1]], dtype=float)

        return np.dot(F, x)

    def hx(x):
        return np.array([x[0], x[2]])


    dt = 0.1
    points = MerweScaledSigmaPoints(4, .1, 2., -1)
    kf = UKF(dim_x=4, dim_z=2, dt=dt, fx=fx, hx=hx, points=points)


    kf.x = np.array([-1., 1., -1., 1])
    kf.P*=0.0001
    #kf.R *=0
    #kf.Q

    zs = []
    for i in range(20):
        z = np.array([i+randn()*0.1, i+randn()*0.1])
        zs.append(z)



    Ms, Ps = kf.batch_filter(zs)
    smooth_x, _, _ = kf.rts_smoother(Ms, Ps, dt=dt)

    if DO_PLOT:
        zs = np.asarray(zs)

        #plt.plot(zs[:,0])
        plt.plot(Ms[:,0])
        plt.plot(smooth_x[:,0], smooth_x[:,2])

        print(smooth_x)


def test_batch_missing_data():
    """ batch filter should accept missing data with None in the measurements """


    def fx(x, dt):
        F = np.array([[1, dt, 0, 0],
                      [0,  1, 0, 0],
                      [0, 0,  1, dt],
                      [0, 0, 0,  1]], dtype=float)

        return np.dot(F, x)

    def hx(x):
        return np.array([x[0], x[2]])


    dt = 0.1
    points = MerweScaledSigmaPoints(4, .1, 2., -1)
    kf = UKF(dim_x=4, dim_z=2, dt=dt, fx=fx, hx=hx, points=points)


    kf.x = np.array([-1., 1., -1., 1])
    kf.P*=0.0001

    zs = []
    for i in range(20):
        z = np.array([i+randn()*0.1, i+randn()*0.1])
        zs.append(z)

    zs[2] = None
    Rs = [1]*len(zs)
    Rs[2] = None
    Ms, Ps = kf.batch_filter(zs)


def test_rts():
    def fx(x, dt):
        A = np.eye(3) + dt * np.array ([[0, 1, 0],
                                        [0, 0, 0],
                                        [0, 0, 0]])
        f = np.dot(A, x)
        return f

    def hx(x):
        return np.sqrt (x[0]**2 + x[2]**2)

    dt = 0.05

    sp = JulierSigmaPoints(n=3, kappa=1.)
    kf = UKF(3, 1, dt, fx=fx, hx=hx, points=sp)

    kf.Q *= 0.01
    kf.R = 10
    kf.x = np.array([0., 90., 1100.])
    kf.P *= 100.
    radar = RadarSim(dt)

    t = np.arange(0,20+dt, dt)

    n = len(t)

    xs = np.zeros((n,3))

    random.seed(200)
    rs = []
    #xs = []
    for i in range(len(t)):
        r = radar.get_range()
        #r = GetRadar(dt)
        kf.predict()
        kf.update(z=[r])

        xs[i,:] = kf.x
        rs.append(r)


    kf.x = np.array([0., 90., 1100.])
    kf.P = np.eye(3)*100
    M, P = kf.batch_filter(rs)
    assert np.array_equal(M, xs), "Batch filter generated different output"

    Qs = [kf.Q]*len(t)
    M2, P2, K = kf.rts_smoother(Xs=M, Ps=P, Qs=Qs)


    if DO_PLOT:
        print(xs[:,0].shape)

        plt.figure()
        plt.subplot(311)
        plt.plot(t, xs[:,0])
        plt.plot(t, M2[:,0], c='g')
        plt.subplot(312)
        plt.plot(t, xs[:,1])
        plt.plot(t, M2[:,1], c='g')
        plt.subplot(313)

        plt.plot(t, xs[:,2])
        plt.plot(t, M2[:,2], c='g')


def test_fixed_lag():
    def fx(x, dt):
        A = np.eye(3) + dt * np.array ([[0, 1, 0],
                                        [0, 0, 0],
                                        [0, 0, 0]])
        f = np.dot(A, x)
        return f

    def hx(x):
        return np.sqrt (x[0]**2 + x[2]**2)

    dt = 0.05

    sp = JulierSigmaPoints(n=3, kappa=0)

    kf = UKF(3, 1, dt, fx=fx, hx=hx, points=sp)

    kf.Q *= 0.01
    kf.R = 10
    kf.x = np.array([0., 90., 1100.])
    kf.P *= 1.
    radar = RadarSim(dt)

    t = np.arange(0,20+dt, dt)

    n = len(t)

    xs = np.zeros((n,3))

    random.seed(200)
    rs = []
    #xs = []

    M = []
    P = []
    N =10
    flxs = []
    for i in range(len(t)):
        r = radar.get_range()
        #r = GetRadar(dt)
        kf.predict()
        kf.update(z=[r])

        xs[i,:] = kf.x
        flxs.append(kf.x)
        rs.append(r)
        M.append(kf.x)
        P.append(kf.P)
        print(i)
        #print(i, np.asarray(flxs)[:,0])
        if i == 20 and len(M) >= N:
            try:
                M2, P2, K = kf.rts_smoother(Xs=np.asarray(M)[-N:], Ps=np.asarray(P)[-N:])
                flxs[-N:] = M2
                #flxs[-N:] = [20]*N
            except:
                print('except', i)
            #P[-N:] = P2


    kf.x = np.array([0., 90., 1100.])
    kf.P = np.eye(3)*100
    M, P = kf.batch_filter(rs)

    Qs = [kf.Q]*len(t)
    M2, P2, K = kf.rts_smoother(Xs=M, Ps=P, Qs=Qs)


    flxs = np.asarray(flxs)
    print(xs[:,0].shape)

    plt.figure()
    plt.subplot(311)
    plt.plot(t, xs[:,0])
    plt.plot(t, flxs[:,0], c='r')
    plt.plot(t, M2[:,0], c='g')
    plt.subplot(312)
    plt.plot(t, xs[:,1])
    plt.plot(t, flxs[:,1], c='r')
    plt.plot(t, M2[:,1], c='g')

    plt.subplot(313)
    plt.plot(t, xs[:,2])
    plt.plot(t, flxs[:,2], c='r')
    plt.plot(t, M2[:,2], c='g')



def test_circle():
    from filterpy.kalman import KalmanFilter
    from math import radians
    def hx(x):
        radius = x[0]
        angle = x[1]
        x = cos(radians(angle)) * radius
        y = sin(radians(angle)) * radius
        return np.array([x, y])

    def fx(x, dt):
        return np.array([x[0], x[1]+x[2], x[2]])

    std_noise = .1

    sp = JulierSigmaPoints(n=3, kappa=0.)
    f = UKF(dim_x=3, dim_z=2, dt=.01, hx=hx, fx=fx, points=sp)
    f.x = np.array([50., 90., 0])
    f.P *= 100
    f.R = np.eye(2)*(std_noise**2)
    f.Q = np.eye(3)*.001
    f.Q[0,0]=0
    f.Q[2,2]=0

    kf = KalmanFilter(dim_x=6, dim_z=2)
    kf.x = np.array([50., 0., 0, 0, .0, 0.])

    F = np.array([[1., 1., .5, 0., 0., 0.],
                  [0., 1., 1., 0., 0., 0.],
                  [0., 0., 1., 0., 0., 0.],
                  [0., 0., 0., 1., 1., .5],
                  [0., 0., 0., 0., 1., 1.],
                  [0., 0., 0., 0., 0., 1.]])

    kf.F = F
    kf.P*= 100
    kf.H = np.array([[1,0,0,0,0,0],
                     [0,0,0,1,0,0]])


    kf.R = f.R
    kf.Q[0:3, 0:3] = Q_discrete_white_noise(3, 1., .00001)
    kf.Q[3:6, 3:6] = Q_discrete_white_noise(3, 1., .00001)

    measurements = []
    results = []

    zs = []
    kfxs = []
    for t in range (0,12000):
        a = t / 30 + 90
        x = cos(radians(a)) * 50.+ randn() * std_noise
        y = sin(radians(a)) * 50. + randn() * std_noise
        # create measurement = t plus white noise
        z = np.array([x,y])
        zs.append(z)

        f.predict()
        f.update(z)

        kf.predict()
        kf.update(z)

        # save data
        results.append (hx(f.x))
        kfxs.append(kf.x)
        #print(f.x)

    results = np.asarray(results)
    zs = np.asarray(zs)
    kfxs = np.asarray(kfxs)

    print(results)
    if DO_PLOT:
        plt.plot(zs[:,0], zs[:,1], c='r', label='z')
        plt.plot(results[:,0], results[:,1], c='k', label='UKF')
        plt.plot(kfxs[:,0], kfxs[:,3], c='g', label='KF')
        plt.legend(loc='best')
        plt.axis('equal')



def kf_circle():
    from filterpy.kalman import KalmanFilter
    from math import radians
    import math
    def hx(x):
        radius = x[0]
        angle = x[1]
        x = cos(radians(angle)) * radius
        y = sin(radians(angle)) * radius
        return np.array([x, y])

    def fx(x, dt):
        return np.array([x[0], x[1]+x[2], x[2]])


    def hx_inv(x, y):
        angle = math.atan2(y,x)
        radius = math.sqrt(x*x + y*y)
        return np.array([radius, angle])


    std_noise = .1


    kf = KalmanFilter(dim_x=3, dim_z=2)
    kf.x = np.array([50., 0., 0.])

    F = np.array([[1., 0, 0.],
                  [0., 1., 1.,],
                  [0., 0., 1.,]])

    kf.F = F
    kf.P*= 100
    kf.H = np.array([[1,0,0],
                     [0,1,0]])

    kf.R = np.eye(2)*(std_noise**2)
    #kf.Q[0:3, 0:3] = Q_discrete_white_noise(3, 1., .00001)



    zs = []
    kfxs = []
    for t in range (0,2000):
        a = t / 30 + 90
        x = cos(radians(a)) * 50.+ randn() * std_noise
        y = sin(radians(a)) * 50. + randn() * std_noise

        z = hx_inv(x,y)
        zs.append(z)

        kf.predict()
        kf.update(z)

        # save data
        kfxs.append(kf.x)

    zs = np.asarray(zs)
    kfxs = np.asarray(kfxs)


    if DO_PLOT:
        plt.plot(zs[:,0], zs[:,1], c='r', label='z')
        plt.plot(kfxs[:,0], kfxs[:,1], c='g', label='KF')
        plt.legend(loc='best')
        plt.axis('equal')




def two_radar():

    # code is not complete - I was using to test RTS smoother. very similar
    # to two_radary.py in book.

    import numpy as np
    import matplotlib.pyplot as plt

    from numpy import array
    from numpy.linalg import norm
    from numpy.random import randn
    from math import atan2, radians

    from filterpy.common import Q_discrete_white_noise

    class RadarStation(object):

        def __init__(self, pos, range_std, bearing_std):
            self.pos = asarray(pos)

            self.range_std = range_std
            self.bearing_std = bearing_std


        def reading_of(self, ac_pos):
            """ Returns range and bearing to aircraft as tuple. bearing is in
            radians.
            """

            diff = np.subtract(self.pos, ac_pos)
            rng = norm(diff)
            brg = atan2(diff[1], diff[0])
            return rng, brg


        def noisy_reading(self, ac_pos):
            rng, brg = self.reading_of(ac_pos)
            rng += randn() * self.range_std
            brg += randn() * self.bearing_std
            return rng, brg




    class ACSim(object):

        def __init__(self, pos, vel, vel_std):
            self.pos = asarray(pos, dtype=float)
            self.vel = asarray(vel, dtype=float)
            self.vel_std = vel_std


        def update(self):
            vel = self.vel + (randn() * self.vel_std)
            self.pos += vel

            return self.pos

    dt = 1.


    def hx(x):
        r1, b1 = hx.R1.reading_of((x[0], x[2]))
        r2, b2 = hx.R2.reading_of((x[0], x[2]))

        return array([r1, b1, r2, b2])
        pass



    def fx(x, dt):
        x_est = x.copy()
        x_est[0] += x[1]*dt
        x_est[2] += x[3]*dt
        return x_est



    vx, vy = 0.1, 0.1

    f = UnscentedKalmanFilter(dim_x=4, dim_z=4, dt=dt, hx=hx, fx=fx, kappa=0)
    aircraft = ACSim ((100,100), (vx*dt,vy*dt), 0.00000002)


    range_std = 0.001  # 1 meter
    bearing_std = 1/1000 # 1mrad

    R1 = RadarStation ((0,0), range_std, bearing_std)
    R2 = RadarStation ((200,0), range_std, bearing_std)

    hx.R1 = R1
    hx.R2 = R2

    f.x = array([100, vx, 100, vy])

    f.R = np.diag([range_std**2, bearing_std**2, range_std**2, bearing_std**2])
    q = Q_discrete_white_noise(2, var=0.0002, dt=dt)
    #q = np.array([[0,0],[0,0.0002]])
    f.Q[0:2, 0:2] = q
    f.Q[2:4, 2:4] = q
    f.P = np.diag([.1, 0.01, .1, 0.01])


    track = []
    zs = []


    for i in range(int(300/dt)):

        pos = aircraft.update()

        r1, b1 = R1.noisy_reading(pos)
        r2, b2 = R2.noisy_reading(pos)

        z = np.array([r1, b1, r2, b2])
        zs.append(z)
        track.append(pos.copy())

    zs = asarray(zs)


    xs, Ps, Pxz, pM, pP = f.batch_filter(zs)
    ms, _, _ = f.rts_smoother(xs, Ps)

    track = asarray(track)
    time = np.arange(0,len(xs)*dt, dt)

    plt.figure()
    plt.subplot(411)
    plt.plot(time, track[:,0])
    plt.plot(time, xs[:,0])
    plt.legend(loc=4)
    plt.xlabel('time (sec)')
    plt.ylabel('x position (m)')
    plt.tight_layout()



    plt.subplot(412)
    plt.plot(time, track[:,1])
    plt.plot(time, xs[:,2])
    plt.legend(loc=4)
    plt.xlabel('time (sec)')
    plt.ylabel('y position (m)')
    plt.tight_layout()


    plt.subplot(413)
    plt.plot(time, xs[:,1])
    plt.plot(time, ms[:,1])
    plt.legend(loc=4)
    plt.ylim([0, 0.2])
    plt.xlabel('time (sec)')
    plt.ylabel('x velocity (m/s)')
    plt.tight_layout()

    plt.subplot(414)
    plt.plot(time, xs[:,3])
    plt.plot(time, ms[:,3])
    plt.ylabel('y velocity (m/s)')
    plt.legend(loc=4)
    plt.xlabel('time (sec)')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":

    DO_PLOT = True


    test_batch_missing_data()

    #test_linear_2d()
    #test_sigma_points_1D()
    #test_fixed_lag()
    #DO_PLOT = True
    #test_rts()
    #kf_circle()
    #test_circle()


    '''test_1D_sigma_points()
    #plot_sigma_test ()

    x = np.array([[1,2]])
    P = np.array([[2, 1.2],
                  [1.2, 2]])\


    kappa = .1

    xi,w = sigma_points (x,P,kappa)
    xm, cov = unscented_transform(xi, w)'''
    #test_radar()
    #test_sigma_plot()
    #test_julier_weights()
    #test_scaled_weights()

    #print('xi=\n',Xi)
    """
    xm, cov = unscented_transform(Xi, W)
    print(xm)
    print(cov)"""
#    sigma_points ([5,2],9*np.eye(2), 2)

