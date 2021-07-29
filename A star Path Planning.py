import numpy as np
import pymap3d as pm
import math

#거리 계산
# distance = math.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)

## : 넣어야 할 기능
# : 일반 주석

class Obstacle:
    def __init__(self):
        print("-----------장애물 받아오기 시작!-----------")
        self.ob_list = np.array([[2, 2], [2, 5], [3, 2], [5, 9]])
        print("------------장애물 받아오기 끝!------------")

    ##장애물 받아오기
    ##장애물 계속 쌓이지 않게 배열 계속 초기화해주기!!!(어디에선가...)


class PathPlanner:
    def __init__(self, init_pos, init_heading, predict_step, predict_range, move_unit, fin_pos):
        self.edge_points = np.array([[0, 0], [50, 0], [50, 50], [0, 50]])
        self.obstacle = Obstacle()
        self.ob_list = self.obstacle.ob_list


        self.init_pos = np.array(init_pos)
        self.cur_pos = np.array(init_pos) #[x, y] 현재 배가 있는 위치
        self.cur_heading = np.array(init_heading) #[x, y] 벡터!, normalize 수시로 필요!
        # print("init_heading : ")
        # print(self.cur_heading)
        # self.trajectory = np.array([]) #[x, y]
        self.trajectory = np.zeros((1,2))  # [x, y]
        self.fin_pos = fin_pos

        self.predict_step = predict_step
        self.predict_range = predict_range
        self.obstacle_search_range = 1 #이거 파라미터로 전달받기

        self.move_unit = move_unit
        self.goal_range = 1

        self.make_trajectory()

    def is_finished(self):
        if math.sqrt((self.cur_pos[0]-self.fin_pos[0])**2 + (self.cur_pos[1]-self.fin_pos[1])**2) <= self.goal_range:
            return True
        # return np.array_equal(self.cur_pos, self.fin_pos)
        else:
            return False

    def make_trajectory(self):
        search_center = np.array([self.cur_pos]) #numpy인지 자료형 확인 필요
        predict_heading = self.cur_heading

        search_center, predict_heading = self.best_point(search_center, predict_heading)  # 각 값이 잘 들어가 있는지 확인!
        self.trajectory = search_center


        for i in range(self.predict_step - 1):
            search_center, predict_heading = self.best_point(search_center, predict_heading) #각 값이 잘 들어가 있는지 확인!
            #헤딩 어디에 써먹을 건가!
            self.trajectory = np.append(self.trajectory, search_center, axis=0)

    def best_point(self, search_center, predict_heading):
        min_f_value = 1000000
        best_point = np.zeros((0, 2))
        best_heading = np.zeros((0, 2))
        points = self.predict_range * np.array(
            [[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]])
        for i in range(8):
            points[i][0] += search_center[0][0]
            points[i][1] += search_center[0][1]
        # points = search_center + self.predict_range * np.array([[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]])
        for p in points:
            if self.is_line_out(p):
                continue #다른 포인트 찾아 for문으로 돌아가는지 아니면 while 문으로 돌아가는지 확인!
            if self.is_obstacle_in(p):
                continue
            ##8개 다 돌았는데 갈 수 있는 곳 없는 경우 해결!!

            #g값 중 시작부터 거리를 계산함
            # distance_to_cur = math.sqrt((p[0]-self.cur_pos[0])**2 + (p[1]-self.cur_pos[1])**2)
            distance_to_cur = (p[0]-self.cur_pos[0])**2 + (p[1]-self.cur_pos[1])**2

            #g값 중 회전 가중치를 계산함 -- 마이너스로 잘 들어가는지, 180도 이상인 둔각에서는 잘 되는지 필시 확인!
            vec = p - search_center[0]
            # vec = vec / np.linalg.norm(vec)
            vec = vec / math.sqrt((vec[0] ** 2 + vec[1] ** 2))
            sin_theta = round(predict_heading[0] * vec[1] - predict_heading[1] * vec[0], 5)
            theta = math.asin(sin_theta) * 180 / math.pi #degree 값
            # distance_to_search = math.sqrt((p[0] - search_center[0]) ** 2 + (p[1] - search_center[1]) ** 2)
            distance_to_search = (p[0] - search_center[0][0]) ** 2 + (p[1] - search_center[0][1]) ** 2
            rotate_cost = distance_to_search * abs(theta * 1.1)

            #g값 합침
            g = distance_to_cur + rotate_cost

            #h값을 계산함
            h = (p[0] - self.fin_pos[0]) ** 2 + (p[1] - self.fin_pos[1]) ** 2

            #f값 총합함
            f = g + h

            #최소 f 값 찾기
            if min_f_value > f:
                min_f_value = f
                best_point = np.array([p])
                best_heading = vec

        return best_point, best_heading #normalize된 헤딩으로 리턴할 것!

    def is_line_out(self, point):
        # 오른쪽 벡터와 선이 아예 일치할 경우 등 예외처리 꼭 보기!
        crossed = 0
        for i in range(len(self.edge_points)):
            j = (i + 1) % len(self.edge_points)
            if (self.edge_points[i][1] > point[1]) != (self.edge_points[j][1] > point[1]):
                intersection = float((self.edge_points[j][0] - self.edge_points[i][0]) * (
                            point[1] - self.edge_points[i][1]) / (
                                                 self.edge_points[j][1] - self.edge_points[i][1]) +
                                     self.edge_points[i][0])
                if point[0] < intersection:
                    crossed = crossed + 1
        return (crossed % 2) == 0 #밖에 있으면 true

    def is_obstacle_in(self, point):
        for i in range(len(self.ob_list)):
            distance_to_ob = math.sqrt((point[0] - self.ob_list[i][0]) ** 2 + (point[1] - self.ob_list[i][1]) ** 2)
            if distance_to_ob < self.obstacle_search_range:
                return True
        return False

    def move_boat(self):
        print("cur_pos : ", end='')
        print(self.cur_pos)

        vec = self.trajectory[0] - self.cur_pos
        vec = vec / math.sqrt((vec[0]**2 + vec[1]**2))
        next = vec * self.move_unit
        self.cur_pos = self.cur_pos + next
        self.cur_heading = vec

    def obstacle_update(self):
        self.ob_list = self.obstacle.ob_list


def main():
    ## GPS 좌표 처리하는 기능 추가해야 함!
    print("----------경로 추종 클래스 할당------------")
    path_planner = PathPlanner([2, 2], [1, 0], 5, 1.5, 1, [10, 10])
    print("---------경로 추종 클래스 할당 끝----------")

    while not path_planner.is_finished():
        print("--------------아직 도착 안함!--------------")
        print("--------------경로 추종 시작!--------------")
        path_planner.make_trajectory()
        print("---------------경로 추종 끝!---------------")
        path_planner.move_boat()
        print("---------------배 이동 완료!---------------")

    print("-------------최종 목적지 도착!-------------")

if __name__ == '__main__':
    main()
