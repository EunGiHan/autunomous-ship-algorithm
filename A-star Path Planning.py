import time
import numpy as np
import math
import matplotlib.pyplot as plt
import pymap3d as pm


class Map:
    def __init__(self):
        # self.edge_points = edge_points
        self.GPS_edge_points = np.array([[37.44800308207386, 126.65339499985154, 20],
                                [37.44795676716625, 126.65349826489695, 20],
                                [37.44895013580954, 126.65416948770105, 20],
                                [37.44899059427365, 126.654078292596, 20]])
        self.ENU_edge_points = np.zeros_like(self.GPS_edge_points)
        self.limit_lines = []

    def GPS_to_ENU(self):
        for i in range(len(self.GPS_edge_points)):
            self.ENU_edge_points[i][0], self.ENU_edge_points[i][1] , self.ENU_edge_points[i][2] = \
                pm.geodetic2enu(self.GPS_edge_points[i][0], self.GPS_edge_points[i][1], self.GPS_edge_points[i][2], self.GPS_edge_points[0][0], self.GPS_edge_points[0][1], self.GPS_edge_points[0][2])
        self.ENU_edge_points = np.delete(self.ENU_edge_points, 2, axis=1)

    def make_limit_lines(self):
        for i in range(len(self.ENU_edge_points)):
            p1 = self.ENU_edge_points[i]
            p2 = self.ENU_edge_points[(i + 1) % len(self.ENU_edge_points)]
            a = float((p2[1] - p1[1]) / (p2[0] - p1[0]))
            b = float(p2[1] - p2[0] * ((p2[1] - p1[1]) / (p2[0] - p1[0])))
            self.limit_lines.append([a, b])

    def check_line_in(self, point):
        # 특정 점이 밖으로 나갔는지 체크. 직선의 방정식 넘어갔나 넘어가지 않았나 판단
        # true 이면 안에 있는 것
        return True


class Obstacles:
    def __init__(self):
        self.circle_obstacles = []
        self.wall_obstacles = []
        self.limit_line = None

    def check_safety(self, point):
        #안전하면 true 리턴

        #2-1. Circle 장애물 : (갈 지점과 장애물 중심점 사이 거리) > (장애물 반지름 + 안전거리) 일 때만 필터링
        #2-2. Wall 장애물 : min(갈 지점과 start점 사이 거리, 갈 지점과 end 점 사이 거리, 벽 중심까지 거리) > (안전거리) 일 때만 필터링
        return True

class A_star:
    def __init__(self):
        self.current_position = [4, 6] #현재 위치
        self.trajectory = [] #예측 경로
        self.path_length = 0.0
        self.ideal_heading = 0.0
        self.final_goal = [18, 21]
        self.predict_step = 5
        self.obstacles = Obstacles() #잘 작동하는지 확인
        self.limit_lines = Map()



    def make_trajectory(self):
        for i in range(self.predict_step):
            self.search_points()

    def search_points(self): #계속 반복할 함수
        # points = np.array([[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]])  # 내장함수 이용 더 쉽게?
        # points = list(map(self.possible_point, np.array([p + self.current_position for p in points])))
        points = [[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]]  # 내장함수 이용 더 쉽게?
        points = [[p[0] + self.current_position[0], p[1] + self.current_position[1]] for p in points]
        points = [p for p in points if self.possible_point(p)] #갈 수 있는 점만 남겨둠

        for p in points:
            g, delta_heading = self.calc_g(p)
            h = calc_dist(p, self.final_goal)
            f = g + h
            p.extend([f, delta_heading]) #ideal_에 델타 더해준 값을 append 해야 할까.. // point는 [x, y, f, delta_heading]쌍이 됨

        min_f = points[0][2]
        best_step = points[0]
        for p in points:
            if min_f > p[2]:
                best_step = p

        self.trajectory.append(best_step)

    def possible_point(self, point):
        if self.limit_lines.check_line_in(point) and self.obstacles.check_safety(point):
            return True
        else:
            return False

    def calc_g(self, point):
        theta = calc_theta(self.ideal_heading, self.current_position, point)
        if -30 <= theta <= 30:
            return [1, theta]
            #작은 값
        elif -60 < theta < 60:
            return [1, theta]
            #중간 값
        else:
            return [1, theta]
            #큰 값


def calc_theta(heading, boat_position, point):
    return 5

def calc_dist(p1, p2):
    return float(math.sqrt(math.pow(p1[0]-p2[0], 2) + math.pow(p1[1]-p2[1], 2)))

def GPS_to_ENU(origin, point):
    point[0], point[1], point[2] = pm.geodetic2enu(point[0], point[1], 20, origin[0], origin[1], origin[2])
    return point[0:2] #x랑 y만 들어가나 확인


"""
<실제 이동>
1. trajectory 다음 위치로 이동했을 때 bearing과 좌표 고려해 그 다음 배열 위치로 servo, thruster에 Publish
(Q) 직진이나 작은 각도로 갈 때는 속도를 높일까?
"""

def main():
    A_star_path = A_star()

    boat_position = GPS_to_ENU([37.447993741718825, 126.65348921639587]) #GPS로 받아온 시작위치
    final_goal = GPS_to_ENU([37.44849521860946, 126.65380303484946]) #GPS로 찍은 종점 위치
    real_trajectory = boat_position #할당 받아야 함 // 처음엔 첫 위치? // 한 행에 열이 3~4개인데?
    A_star_path.make_trajectory() #처음 작동 잘 되나 확인

    while True:
        if boat_position==real_trajectory[0]: #정확히 그 위치로 못 간다면?
            #해당 지점에 도착했다면
            if boat_position==final_goal:#정확히 그 위치로 못 간다면?
                #최종 도착
                break
            else:
                real_trajectory.remove(0) #마지막 목적지에선 오류나는지 체크
                A_star_path.current_position = real_trajectory[0]

        A_star_path.make_trajectory()
        real_trajectory = A_star_path.trajectory
        boat_position = real_trajectory[0] #이동하기
        time.sleep(0.5)


if __name__ == '__main__':
    main()
