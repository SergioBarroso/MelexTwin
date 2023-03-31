import numpy as np
# Build rotation matrix
import utm
from pyproj import Proj, transform, Transformer, Geod

class LatLongCoppelia:
    def __init__(self, theta= 0.24906021553765, phi = 0, fixedX = -371895.97221695725, fixedY = 4417962.616456918 ):
        self.theta = theta
        self.phi = phi
        rot = np.array([
            [np.cos(self.theta), -np.sin(self.theta), 0.0, 0.0],
            [np.sin(self.theta), np.cos(self.theta), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0], ])

        # Build shear/skew matrix
        m = np.tan(self.phi)
        skew = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [m, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0], ])

        # get affine transform
        self.a = rot @ skew
        print(self.a)

        # Build pipeline
        self.pt_transform = Transformer.from_pipeline(
            f"+proj=pipeline "
            # f"+step +proj=affine +xoff={origin_x} +yoff={origin_y} "
            f"+step +proj=affine +xoff={0} +yoff={0} "
            f"+s11={self.a[0, 0]} +s12={self.a[0, 1]} +s13={self.a[0, 2]} "
            f"+s21={self.a[1, 0]} +s22={self.a[1, 1]} +s23={self.a[1, 2]} "
            f"+s31={self.a[2, 0]} +s32={self.a[2, 1]} +s33={self.a[2, 2]} ")

        self.fixedX = fixedX  # -371886.67816326916
        self.fixedY = fixedY  # 4417967.8278194275

    def __del__(self):
        pass


    def convert_coords(self, lat, long):
        u = utm.from_latlon(lat, long)
        mapX, mapY = self.pt_transform.transform(u[0], u[1])
        mapX = mapX * 1000 - (self.fixedX * 1000)
        mapY = mapY * 1000 - (self.fixedY * 1000)
        coordenadas = [mapX, mapY]
        return coordenadas