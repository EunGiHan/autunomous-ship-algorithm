import numpy as np
import math
import matplotlib.pyplot as plt


class A_star:
    def __init__(self):
        self.current_position = [4, 6] #현재 위치
        # self.current_step = 0 #멈추는 걸 Main에서 관리할까?
        self.trajectory = [] #예측 경로
        self.path_length = 0.0
        self.ideal_heading = 0.0
        self.final_goal = [18, 21]
        self.predict_step = 5

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
        #장애물 있는지 판단
        """
        2. 8방위 점 중 장애물/벽 좌표와 위치가 같거나 안전범위에 [겹침  4번 / 겹치지 않음  3번]
        2-1. Circle 장애물 : (갈 지점과 장애물 중심점 사이 거리) > (장애물 반지름 + 안전거리) 일 때만 필터링
        2-2. Wall 장애물 : min(갈 지점과 start점 사이 거리, 갈 지점과 end 점 사이 거리, 벽 중심까지 거리) > (안전거리) 일 때만 필터링
        2-3. 벽 : 직선의 방정식 넘어갔나 넘어가지 않았나 판단

        """
        return True

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

"""
<실제 이동>
1. trajectory 다음 위치로 이동했을 때 bearing과 좌표 고려해 그 다음 배열 위치로 servo, thruster에 Publish
(Q) 직진이나 작은 각도로 갈 때는 속도를 높일까?
"""

class Boat:
    def __init__(self):
        self.boat_position = [0, 0]


# def check_arrival():


def main():
    A_star_path = A_star()
    boat_position = [0, 0]
    final_goal = [18, 21]

    while not boat_position==final_goal:
        A_star_path.search_points() #계속 반복 안 되게??
        trajectory = A_star_path.trajectory
        # 아래 순서 이상해.. DWA 참고하자
        # for t in trajectory:
        #     # 목표점을 t로 하여 servo, thruster 퍼블리시
        #     if boat_position == t[0, 1]:
        #         continue
        #     else:
        #         

    # goal = Goal()
    # goal.get_xy()
    # dwa_path = DWA_Calc()
    #
    # while not rospy.is_shutdown():
    #     goal.print_goal_in_terminal()
    #     timer = timer - rospy.Duration(1)
    #     if timer <= rospy.Duration(0):
    #         if len(goal.way_list) == 0:
    #             break
    #         elif len(goal.way_list) == 1:
    #             dwa_path.goal_x = goal.way_list[0][0]
    #             dwa_path.goal_y = goal.way_list[0][1]
    #             goal.set_next_point()
    #             timer = rospy.Duration(3)
    #         else:
    #             goal.set_next_point()
    #             dwa_path.goal_x = goal.way_list[0][0]
    #             dwa_path.goal_y = goal.way_list[0][1]
    #             timer = rospy.Duration(3)


if __name__ == '__main__':
    main()