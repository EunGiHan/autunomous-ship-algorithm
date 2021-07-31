import numpy as np
import math
import matplotlib.pyplot as plt
import sys
from PyQt5.QtWidgets import QApplication, QDialog, QGraphicsScene
from PyQt5.QtGui import QPen
from PyQt5.Qt import QLineF

from "A star Simulation Window" import UI_Simulator


class Obstacle:
    def __init__(self):
        self.ob_list = np.array([[2, 2], [2, 5], [3, 2], [5, 9]])


class PathPlanner:
    def __init__(self):
        ####이미 지난 path 저장하기!!!!
        self.edge_points = np.array([[0, 0], [50, 0], [50, 50], [0, 50]])
        self.obstacle = Obstacle()
        self.ob_list = self.obstacle.ob_list

        self.init_pos = np.array([2, 2])
        self.cur_pos = self.init_pos #[x, y] 현재 배가 있는 위치
        self.cur_heading = np.array([1, 0]) #[x, y] 벡터!, normalize 수시로 필요!
        self.trajectory = np.zeros((1,2))  # [x, y]
        self.end_pos = [20, 20]

        self.predict_step = 5
        self.predict_step_size = 1.5
        self.obstacle_search_range = 1

        self.move_size = 1
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

        self.trajectory = search_center

        for i in range(self.predict_step - 1):
            search_center, predict_heading = self.best_point(search_center, predict_heading)  # 각 값이 잘 들어가 있는지 확인!
            self.trajectory = np.append(self.trajectory, search_center, axis=0)

    def best_point(self, search_center, predict_heading):
        min_f_value = 1000000
        best_point = np.zeros((0, 2))
        best_heading = np.zeros((0, 2))
        points = self.predict_step_size * np.array(
            [[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]])
        for i in range(8): #더 좋은 방법 있나 찾기
            points[i][0] += search_center[0][0]
            points[i][1] += search_center[0][1]

        for p in points:
            if self.is_line_out(p):
                continue  # 다른 포인트 찾아 for문으로 돌아가는지 아니면 while 문으로 돌아가는지 확인!
            if self.is_obstacle_in(p):
                continue

            # g값 중 시작부터 거리를 계산함
            distance_to_cur = (p[0] - self.cur_pos[0]) ** 2 + (p[1] - self.cur_pos[1]) ** 2

            # g값 중 회전 가중치를 계산함 -- 마이너스로 잘 들어가는지, 180도 이상인 둔각에서는 잘 되는지 필시 확인!
            vec = p - search_center[0]
            vec = vec / math.sqrt((vec[0] ** 2 + vec[1] ** 2))
            sin_theta = round(predict_heading[0] * vec[1] - predict_heading[1] * vec[0], 5)
            theta = math.asin(sin_theta) * 180 / math.pi  # degree 값

            distance_to_search = (p[0] - search_center[0][0]) ** 2 + (p[1] - search_center[0][1]) ** 2
            rotate_cost = distance_to_search * abs(theta * 1.1)

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

    def is_line_out(self, point):  # 오른쪽 벡터와 선이 아예 일치할 경우 등 예외처리 꼭 보기!
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

    def obstacle_update(self):
        self.ob_list = self.obstacle.ob_list


class MainController:
    def __init__(self):
        self.path_planner = PathPlanner()

    def run_to_goal(self):
        while not self.path_planner.is_finished():
            self.path_planner.make_trajectory()
            self.path_planner.move_boat()

class SimulationWindow(QDialog):

    controller = MainController()

    def __init__(self, parent=None):
        super(SimulationWindow, self).__init__(parent)
        self.ui = UI_Simulator()
        self.ui.set_UI(self)

        ##Simulator Initial Values
        self.ui.start_point_x_spinBox.setValue(2)
        self.ui.start_point_y_spinBox.setValue(2)
        self.ui.start_heading_x_spinBox.setValue(0)
        self.ui.start_heading_y_spinBox.setValue(0)
        self.ui.end_point_x_spinBox.setValue(20)
        self.ui.end_point_y_spinBox.setValue(20)
        self.ui.ob_search_range_doubleSpinBox.setValue(1)
        self.ui.predict_step_spinBox.setValue(5)
        self.ui.predict_step_size_doubleSpinBox.setValue(1.5)
        self.ui.move_size_doubleSpinBox.setValue(1)
        self.ui.arrival_range_doubleSpinBox.setValue(1)
        # self.ui.status_lineEdit.setValue(1) #흠??????????//

    def do_calc(self):
        # if (self.goal_flag == False):
        #     self.goal_flag = self.controller.run_to_goal(self.time_step, self.goal_flag)
        # self.time_step += 1
        #
        # if (self.goal_flag == True):
        #     self.timer.stop()

        self.controller.path_planner.init_pos = np.array([self.ui.start_point_x_spinBox.value(), self.ui.start_point_y_spinBox.value()]) #잘 들어가나?
        self.controller.path_planner.cur_pos = self.controller.path_planner.init_pos
        self.controller.path_planner.cur_heading = np.array(
            [self.ui.start_heading_x_spinBox.value(), self.ui.start_heading_y_spinBox.value()])
        # self.trajectory = np.zeros((1, 2))  #이거 설정해줘야 하나??
        self.controller.path_planner.end_pos = np.array(
            [self.ui.end_point_x_spinBox.value(), self.ui.end_point_y_spinBox.value()])
        self.controller.path_planner.predict_step = self.ui.predict_step_spinBox.value()
        self.controller.path_planner.predict_step_size = self.ui.predict_step_size_doubleSpinBox.value()
        self.controller.path_planner.obstacle_search_range = self.ui.ob_search_range_doubleSpinBox.value()
        self.controller.path_planner.move_size = self.ui.move_size_doubleSpinBox.value()
        self.controller.path_planner.goal_range = self.ui.arrival_range_doubleSpinBox.value()

    def start(self):
        #흠.....
        self.timer.timeout.connect(self.do_calc)
        self.timer.start(50)

    def pause(self):
        self.timer.stop()

    def reset(self):
        self.controller.path_planner.init_pos =  np.zeros((1, 2)) #이거 맞나?
        self.controller.path_planner.cur_pos.clear()
        self.controller.path_planner.cur_heading.clear()
        self.trajectory = np.zeros((1, 2))  #이거 설정해줘야 하나??
        self.controller.path_planner.end_pos = np.array(
            [self.ui.end_point_x_spinBox.value(), self.ui.end_point_y_spinBox.value()])
        self.controller.path_planner.predict_step = self.ui.predict_step_spinBox.value()
        self.controller.path_planner.predict_step_size = self.ui.predict_step_size_doubleSpinBox.value()
        self.controller.path_planner.obstacle_search_range = self.ui.ob_search_range_doubleSpinBox.value()
        self.controller.path_planner.move_size = self.ui.move_size_doubleSpinBox.value()
        self.controller.path_planner.goal_range = self.ui.arrival_range_doubleSpinBox.value()

    def draw(self, trajectory, start_pos, past_path, obstacles, end_pos):
        scale = 0.05 #1pixel = 0.05m
        c = 1/scale
        X_offset = 100 #나중에 다시 보기!!
        Y_offset = 500 #나중에 다시 보기!!

        self.scene = GraphicsScene()
        self.ui.graphicsView.setScene(self.scene)

        ## 축 그리기
        pen_axis = QPen(Qt.Black)
        pen_axis.setWidth(2)
        #선 위치 살펴보기
        self.scene.addLine(QLineF(X_offset, Y_offset, X_offset, X_offset), pen_axis) #왼쪽 세로
        self.scene.addLine(QLineF(X_offset, Y_offset, Y_offset, Y_offset), pen_axis) #위쪽 가로
        self.scene.addLine(QLineF(Y_offset, Y_offset, Y_offset, X_offset), pen_axis) #오른쪽 세로
        self.scene.addLine(QLineF(Y_offset, X_offset, X_offset, X_offset), pen_axis) #아래쪽 가로

        ## 예측 경로 그리기
        pen_trajectory = QPen(Qt.red)
        for i in range(len(trajectory)-1):
            self.scene.addLine(QLineF(c * trajectory[i][0] + X_offset, c*trajectory[i][1] + Y_offset,
                                      c*trajectory[i+1][0] + X_offset, c*trajectory[i+1][1] + Y_offset), pen_trajectory)

        ## 장애물 그리기
        pen_obstacle = QPen(Qt.Black)
        for i in range(len(obstacles)):
            obstacle_radius = c * 0.3
            self.scene.addEllipse(c * obstacles[i][0] + X_offset - obstacle_radius,
                                  -c * obstacles[i][1] + Y_offset - obstacle_radius, obstacle_radius*2,
                                  obstacle_radius*2, pen_obstacle, QBrush(Qt.Black))
            
        ## 로봇 그리기
        ## 헤딩 그리기
        ## 지난 경로 그리기
        
        ## 출발지점 그리기
        pen_start = QPen(Qt.Red)
        diameter = c * 0.6
        self.scene.addEllipse(c * start_pos[0] + X_offset - diameter / 2,
                              -c * start_pos[1] + Y_offset - diameter / 2, diameter, diameter, pen_start,
                              QBrush(Qt.green))
        ## 목적지 그리기
        pen_end = QPen(Qt.blue)
        diameter = c * 0.6
        self.scene.addEllipse(c * end_pos[0] + X_offset - diameter / 2,
                              -c * end_pos[1] + Y_offset - diameter / 2, diameter, diameter, pen_end,
                              QBrush(Qt.green))



class GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, 0, 0, 600, 600, parent=None) #위치 재조정 필요!
        self.opt = "" #얘 정체가 뭐지???

    def mousePressEvent(self, event):
        global goal_from_picture_x, goal_from_picture_y #흠....

        goal_from_picture_x = (event.scenePos().x() - 100) * 0.05
        goal_from_picture_y = -(event.scenePos().y() - 500) * 0.05


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = SimulationWindow()
    sys.exit(app.exec_())
    main()
