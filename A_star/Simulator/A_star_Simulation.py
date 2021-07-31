# -*- coding: utf-8 -*-

import numpy as np
import math
import sys

from PyQt5.QtWidgets import QApplication, QDialog, QGraphicsScene, QStatusBar
from PyQt5.QtGui import QPen, QBrush
from PyQt5.Qt import QTimer
from PyQt5.QtCore import Qt, QLineF
from PyQt5.QtTest import QTest

from A_star_Simulation_Window import UI_Simulator

"""
수정해야 할 것 :
1) 특정 조건에서 Stuck 되는 것 : 알고리즘 문제인지, gain 값 문제인지 확인! -> Stuck 되었을 때 처리하는 코드 추가
2) Simulator에 Gain 값 조정할 수 있게 설정
3) 초깃값 여러군데 통일
4) Reset, Pause 기능 잘 안 되는 것
5) 초기 heading이 잘 들어가는지 확인
6) obstacle, 시작점, 도착점도 마우스로 찍어서 설정할 수 있게 하는 기능
7) heading을 변경했을 때 normalize해야 함
8) 넘파이 배열을 스마트하게 쓸 수 있는 방법 이용
9) 헤딩 회전에서 각도가 마이너스로 잘 들어가는지, 180도 이상인 둔각에서는 잘 되는지 필시 확인!
10) 휴리스틱에도 gain 값 곱해주는 것 고려
11) lineout 함수에서 오른쪽 벡터와 선이 아예 일치할 경우 등 예외처리 꼭 보기!
12) 헤딩 방향으로 장애물 자동 할당 기능
13) 닫기 버튼 누르면 프로세스 종료되게
14) trajectory 그릴 때 마지막 점까지 그릴 수 있도록(?)
15) 시뮬레이터 배 모양으로 바꾸기
16) 시뮬레이터 헤딩 표시하기
17) Status Bar에 마우스 위치 표시
"""


class Obstacle:
    def __init__(self):
        self.ob_list = np.array([[1, 2], [2, 2], [3, 2], [4, 2], [5, 2], [6, 2], [7, 2], [8, 2], [9, 2], [10, 2],
                                 [11, 2], [12, 2], [13, 2], [14, 2], [15, 2], [16, 2], [17, 2], [18, 2], [19, 2], [20, 2],
                                 [19, 6], [20, 6], [21, 6], [22, 6], [23, 6], [24, 6], [25, 6], [26, 6], [27, 6], [28, 6], [29, 6],
                                 [17, 10], [18, 10], [19, 10], [17, 11], [18, 11], [19, 11],
                                 [24, 18], [24, 19], [24, 20], [24, 21], [24, 22], [24, 23]])


class PathPlanner:
    def __init__(self):
        self.edge_points = np.array([[0, 0], [30, 0], [30, 30], [0, 30]])
        self.obstacle = Obstacle()
        self.ob_list = self.obstacle.ob_list

        self.init_pos = np.array([1, 1])
        self.cur_pos = self.init_pos #[x, y] 현재 배가 있는 위치
        self.cur_heading = np.array([1, 0]) #[x, y] 벡터!, normalize 수시로 필요!
        self.trajectory = np.zeros((1,2))  # [x, y]
        self.end_pos = [25, 25]

        self.predict_step = 5
        self.predict_step_size = 1.5
        self.obstacle_search_range = 1

        self.move_size = 1
        self.past_path = np.array([self.init_pos])
        self.goal_range = 1

        self.make_trajectory()

    def is_finished(self):
        if math.sqrt((self.cur_pos[0]-self.end_pos[0])**2 + (self.cur_pos[1]-self.end_pos[1])**2) <= self.goal_range:
            return True
        else:
            return False

    def make_trajectory(self):
        search_center = np.array([self.cur_pos])
        predict_heading = self.cur_heading

        search_center, predict_heading = self.best_point(search_center, predict_heading)

        self.trajectory = search_center #초기화

        for i in range(self.predict_step - 1):
            search_center, predict_heading = self.best_point(search_center, predict_heading)
            self.trajectory = np.append(self.trajectory, search_center, axis=0)

    def best_point(self, search_center, predict_heading):
        temp = 0
        min_f_value = float("Inf")
        best_point = np.zeros((0, 2))
        best_heading = np.zeros((0, 2))
        points = self.predict_step_size * np.array(
            [[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]])
        for i in range(8): #더 좋은 방법 있나 찾기
            points[i][0] += search_center[0][0]
            points[i][1] += search_center[0][1]

        for p in points:
            if self.is_line_out(p):
                continue
            if self.is_obstacle_in(p):
                continue

            # g값 중 시작부터 거리를 계산함
            distance_to_cur = (p[0] - self.cur_pos[0]) ** 2 + (p[1] - self.cur_pos[1]) ** 2

            # g값 중 회전 가중치를 계산함
            vec = p - search_center[0]
            vec = vec / math.sqrt((vec[0] ** 2 + vec[1] ** 2))
            sin_theta = round(predict_heading[0] * vec[1] - predict_heading[1] * vec[0], 5)
            theta = math.asin(sin_theta) * 180 / math.pi  # degree 값

            if 45 >= abs(theta):
                rotate_cost_gain = 1
            # elif 60 >= abs(theta):
            #     rotate_cost_gain = 1.2
            elif 90 >= abs(theta):
                rotate_cost_gain = 1.2
            else:
                rotate_cost_gain = 2

            distance_to_search = (p[0] - search_center[0][0]) ** 2 + (p[1] - search_center[0][1]) ** 2
            rotate_cost = distance_to_search * rotate_cost_gain

            # g값 합침
            g = distance_to_cur + rotate_cost

            # h값을 계산함
            h = (p[0] - self.end_pos[0]) ** 2 + (p[1] - self.end_pos[1]) ** 2

            # f값 총합
            f = g + h

            # 최소 f 값 찾기
            if min_f_value > f:
                min_f_value = f
                best_point = np.array([p])
                best_heading = vec

        return best_point, best_heading

    def is_line_out(self, point):
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
        next = vec * self.move_size
        
        self.cur_pos = self.cur_pos + next
        self.cur_heading = vec
        self.past_path = np.append(self.past_path, np.array([self.cur_pos]), axis=0)

    def obstacle_update(self): #lidar로 받아올 때 혹은 장애물 실시간 할당 시
        self.ob_list = self.obstacle.ob_list


class MainController:
    def __init__(self):
        self.path_planner = PathPlanner()

    def run_to_goal(self):
        while not self.path_planner.is_finished():
            self.path_planner.make_trajectory()
            self.path_planner.move_boat()
            window.draw(self.path_planner.edge_points, self.path_planner.cur_pos, self.path_planner.trajectory, self.path_planner.init_pos,
                        self.path_planner.past_path, self.path_planner.ob_list, self.path_planner.end_pos)
            QTest.qWait(100)
        print("----- MainController/run_to_goal/finished")
        window.timer.stop()


class SimulationWindow(QDialog):
    controller = MainController()
    timer = QTimer()

    def __init__(self, parent=None):
        super(SimulationWindow, self).__init__(parent)
        self.ui = UI_Simulator()
        self.ui.set_UI(self)

        ##Simulator Initial Values
        self.ui.start_point_x_spinBox.setValue(1)
        self.ui.start_point_y_spinBox.setValue(1)
        self.ui.start_heading_x_spinBox.setValue(0)
        self.ui.start_heading_y_spinBox.setValue(0)
        self.ui.end_point_x_spinBox.setValue(20)
        self.ui.end_point_y_spinBox.setValue(20)
        self.ui.ob_search_range_doubleSpinBox.setValue(1)
        self.ui.predict_step_spinBox.setValue(5)
        self.ui.predict_step_size_doubleSpinBox.setValue(1.5)
        self.ui.move_size_doubleSpinBox.setValue(1)
        self.ui.arrival_range_doubleSpinBox.setValue(1)
        self.ui.status_lineEdit.setText("Init")

    def do_calc(self):
        self.controller.path_planner.init_pos = np.array([self.ui.start_point_x_spinBox.value(), self.ui.start_point_y_spinBox.value()]) #잘 들어가나?
        self.controller.path_planner.cur_pos = self.controller.path_planner.init_pos
        self.controller.path_planner.cur_heading = np.array(
            [self.ui.start_heading_x_spinBox.value(), self.ui.start_heading_y_spinBox.value()])
        self.trajectory = np.zeros((1, 2))
        self.controller.path_planner.past_path = np.array([self.controller.path_planner.init_pos])
        self.controller.path_planner.end_pos = np.array(
            [self.ui.end_point_x_spinBox.value(), self.ui.end_point_y_spinBox.value()])
        self.controller.path_planner.predict_step = self.ui.predict_step_spinBox.value()
        self.controller.path_planner.predict_step_size = self.ui.predict_step_size_doubleSpinBox.value()
        self.controller.path_planner.obstacle_search_range = self.ui.ob_search_range_doubleSpinBox.value()
        self.controller.path_planner.move_size = self.ui.move_size_doubleSpinBox.value()
        self.controller.path_planner.goal_range = self.ui.arrival_range_doubleSpinBox.value()

        self.controller.run_to_goal()

    def start(self):
        self.ui.status_lineEdit.setText("Start")
        self.timer.start(50)
        self.timer.timeout.connect(self.do_calc)

    def pause(self):
        self.ui.status_lineEdit.setText("Pause")
        self.timer.stop()

    def reset(self):
        self.ui.status_lineEdit.setText("Reset")

        # self.controller.path_planner.init_pos = np.zeros((2,)) #이거 맞나?
        # self.controller.path_planner.cur_pos = self.controller.path_planner.init_pos
        # self.controller.path_planner.cur_heading = np.zeros((2,))
        # self.controller.path_planner.trajectory = np.zeros((1, 2))  #이거 설정해줘야 하나??
        # self.controller.path_planner.past_path = np.array([self.controller.path_planner.init_pos])
        # self.controller.path_planner.end_pos = np.zeros((2,))
        # self.controller.path_planner.predict_step = 0
        # self.controller.path_planner.predict_step_size = 0
        # self.controller.path_planner.obstacle_search_range = 0
        # self.controller.path_planner.move_size = 0
        # self.controller.path_planner.goal_range = 0

        self.ui.start_point_x_spinBox.setValue(1)
        self.ui.start_point_y_spinBox.setValue(1)
        self.ui.start_heading_x_spinBox.setValue(0)
        self.ui.start_heading_y_spinBox.setValue(0)
        self.ui.end_point_x_spinBox.setValue(20)
        self.ui.end_point_y_spinBox.setValue(20)
        self.ui.ob_search_range_doubleSpinBox.setValue(1)
        self.ui.predict_step_spinBox.setValue(5)
        self.ui.predict_step_size_doubleSpinBox.setValue(1.5)
        self.ui.move_size_doubleSpinBox.setValue(1)
        self.ui.arrival_range_doubleSpinBox.setValue(1)

    def draw(self, edge_points, cur_pos, trajectory, start_pos, past_path, obstacles, end_pos):
        scale = 0.08 # 1pixel = 0.05m // 작을수록 맵 범위를 벗어남
        c = 1/scale

        self.scene = GraphicsScene()
        self.ui.graphicsView.setScene(self.scene)

        ## 경계선 그리기
        pen_edges = QPen(Qt.black, 2)
        for i in range(len(edge_points)):
            j = (i+1) % len(edge_points)
            self.scene.addLine(QLineF(c * edge_points[i][0], -c * edge_points[i][1],
                                      c * edge_points[j][0], -c * edge_points[j][1]), pen_edges)

        ## 예측 경로 그리기
        pen_trajectory = QPen(Qt.red)
        for i in range(len(trajectory)-1):
            self.scene.addLine(QLineF(c * trajectory[i][0], -c * trajectory[i][1],
                                      c * trajectory[i+1][0], -c * trajectory[i+1][1]), pen_trajectory)

        ## 지난 경로 그리기
        pen_pastPath = QPen(Qt.green)
        if len(past_path) > 1:
            for i in range(len(past_path) - 1):
                self.scene.addLine(QLineF(c * past_path[i][0], -c * past_path[i][1],
                                          c * past_path[i + 1][0], -c * past_path[i + 1][1]), pen_pastPath)

        ## 장애물 그리기
        pen_obstacle = QPen(Qt.black)
        for i in range(len(obstacles)):
            obstacle_diameter = c * 0.4
            self.scene.addEllipse(c * obstacles[i][0] - obstacle_diameter/2, -c * obstacles[i][1] - obstacle_diameter/2,
                                  obstacle_diameter, obstacle_diameter, pen_obstacle, QBrush(Qt.black))

        ## 배 그리기
        pen_boat = QPen(Qt.green)
        diameter = c * 0.4
        self.scene.addEllipse(c * cur_pos[0] - diameter / 2, -c * cur_pos[1] - diameter / 2,
                              diameter, diameter, pen_boat, QBrush(Qt.green))

        ## 출발지점 그리기
        pen_start = QPen(Qt.red)
        diameter = c * 0.4
        self.scene.addEllipse(c * start_pos[0] - diameter / 2, -c * start_pos[1] - diameter / 2,
                              diameter, diameter, pen_start, QBrush(Qt.red))

        ## 목적지 그리기
        pen_end = QPen(Qt.blue)
        diameter = c * 0.4
        self.scene.addEllipse(c * end_pos[0] - diameter / 2, -c * end_pos[1] - diameter / 2,
                              diameter, diameter, pen_end, QBrush(Qt.blue))

        # self.status_bar = self.statusBar() #AttributeError: 'SimulationWindow' object has no attribute 'statusBar'

    # def mouseMoveEvent(self, event):
    #     mouse_pt = "Mouse Point : x={0},y={1}, global={2},{3}".format(event.x(), event.y(), event.globalX(),
    #                                                                   event.globalY())
    #     # self.status_bar().showMessage(mouse_pt)
    #     # print(mouse_pt)
    #     self.ui.status_lineEdit.setText(mouse_pt)


class GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent=None)

    # def mousePressEvent(self, event): #이걸로 써야 할까?
    #     global goal_from_picture_x, goal_from_picture_y
    #
    #     goal_from_picture_x = (event.scenePos().x() - 10) * 0.05
    #     goal_from_picture_y = -(event.scenePos().y() - 50) * 0.05


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SimulationWindow()
    window.show()
    sys.exit(app.exec_())