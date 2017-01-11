#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math


class Camera:
    '''
        Klasse repräsentiert ein Kamera Objekt.
    '''

    def __init__(self, l=0, d=0, h=0, n=480, m=640, gamma=0, theta=0, psi=0, aperture=140):
        '''
            Konstrukor zum Instanziieren eines Kamera Objekts.
        '''
        self.l = l
        self.d = d
        self.h = h
        self.gamma = gamma
        self.theta = theta
        self.psi = psi
        self.aperture = aperture
        self.n = n
        self.m = m

    def image_to_world_coordinates(self, u, v):
        '''
            Methode wandelt 2D Bild Kooridnaten in 3D Weltkoordianten um.
        '''
        x = self.h * math.pow(math.tan(self._calculate_inner_term(self.theta, self.n, u)), -1) + self.l
        y = self.h * math.pow(math.tan(self._calculate_inner_term(self.theta, self.n, u)), -1) * math.sin(self._calculate_inner_term(self.gamma, self.m, v)) + self.d
        z = 0

        return (x, y, z)

    def get_horizon_y(self):
        '''
            Methode liefert den y-Wert des Horizontes zurück.
        '''
        return self.n / 2 + self.aperture / float(self.n) * self.theta

    def _calculate_inner_term(self, angle, dim, pixel):
        return self._deg2rad((angle - self.aperture / 2) + pixel * self.aperture / (float(dim) - 1))

    def _deg2rad(self, x):
        return x * math.pi / 180.0
