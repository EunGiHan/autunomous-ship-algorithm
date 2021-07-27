import pymap3d as pm
import numpy as np

class Map:
    def __init__(self):
        # self.edge_points = edge_points
        self.GPS_edge_points = [[37.44800308207386, 126.65339499985154],
                           [37.44795676716625, 126.65349826489695],
                           [37.44895013580954, 126.65416948770105],
                           [37.44899059427365, 126.654078292596]]
        self.ENU_edge_points = []
        self.limit_lines = []

    def make_limit_lines(self):
        for i in range(len(self.ENU_edge_points)):
            p1 = self.ENU_edge_points[i]
            p2 = self.ENU_edge_points[(i+1)%len(self.ENU_edge_points)]
            a = float((p2[1]-p1[1]) / (p2[0]-p1[0]))
            b = float(p2[1] - p2[0] * ((p2[1]-p1[1])/(p2[0]-p1[0])))
            self.limit_lines.append([a, b])
    
    def check_line_over(self, point):
        #특정 점이 밖으로 나갔는지 체크
        pass


def main():
    # map = Map()
    pass
    

