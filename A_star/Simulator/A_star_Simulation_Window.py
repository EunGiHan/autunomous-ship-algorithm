# -*- coding: utf-8 -*-

from PyQt5 import QtGui
from PyQt5 import QtCore
from PyQt5 import QtWidgets


class UI_Simulator(object):
    def set_UI(self, A_star_Simulator):
        ################ Simulator
        A_star_Simulator.setObjectName("A_star_Simulator")
        A_star_Simulator.resize(1100, 750) #맵 크기에 따라 수정하기

        ################ Widgets
        ### QGraphicView : 시뮬레이터 화면
        self.graphicsView = QtWidgets.QGraphicsView(A_star_Simulator)
        # self.graphicsView.setGeometry(QtCore.QRect(0, 0, 600, 600)) #맵 크기 조정 시 필요할 수도
        self.graphicsView.setObjectName("graphicsView") #수정하기

        ### QLabel : Settings Value
        self.settings_label = QtWidgets.QLabel(A_star_Simulator)
        # font = QtGui.QFont()
        # font.setBold(True)
        # font.setWeight(75)
        # self.settings_label.setFont(QtGui.QFont(pointSize=16, weight=75)) #bold 어떻게 설정?
        self.settings_label.setObjectName("settings_label")

        ### QLabel : Start Point
        self.start_point_label = QtWidgets.QLabel(A_star_Simulator)
        self.start_point_label.setObjectName("start_point_label")

        ### QSpinBox : Start Point(X)
        self.start_point_x_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.start_point_x_spinBox.setMinimum(0)         ###Max도 정하기!!!!!!!!!!!
        self.start_point_x_spinBox.setObjectName("start_point_x_spinBox")

        ### QSpinBox : Start Point(Y)
        self.start_point_y_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.start_point_y_spinBox.setMinimum(0)        ###Max도 정하기!!!!!!!!!!!
        self.start_point_y_spinBox.setObjectName("start_point_y_spinBox")

        ### QLabel : Start Heading
        self.start_heading_label = QtWidgets.QLabel(A_star_Simulator)
        self.start_heading_label.setObjectName("start_heading_label")

        ### QSpinBox : Start Heading(X)
        self.start_heading_x_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.start_heading_x_spinBox.setMinimum(0)        ###Max도 정하기!!!!!!!!!!!
        self.start_heading_x_spinBox.setObjectName("start_heading_x_spinBox")

        ### QSpinBox : Start Point(Y)
        self.start_heading_y_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.start_heading_y_spinBox.setMinimum(0)        ###Max도 정하기!!!!!!!!!!!
        self.start_heading_y_spinBox.setObjectName("start_heading_y_spinBox")

        ### QLabel : End Point
        self.end_point_label = QtWidgets.QLabel(A_star_Simulator)
        self.end_point_label.setObjectName("end_point_label")

        ### QSpinBox : End Point(X)
        self.end_point_x_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.end_point_x_spinBox.setMinimum(0)        ###Max도 정하기!!!!!!!!!!!
        self.end_point_x_spinBox.setObjectName("end_point_x_spinBox")

        ### QSpinBox : End Point(Y)
        self.end_point_y_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.end_point_y_spinBox.setMinimum(0)        ###Max도 정하기!!!!!!!!!!!
        self.end_point_y_spinBox.setObjectName("end_point_y_spinBox")

        ### QLabel : Obstacle Search Range
        self.ob_search_range_label = QtWidgets.QLabel(A_star_Simulator)
        self.ob_search_range_label.setObjectName("ob_search_range_label")

        ### QSpinBox : Obstacle Search Range
        self.ob_search_range_doubleSpinBox = QtWidgets.QDoubleSpinBox(A_star_Simulator)
        self.ob_search_range_doubleSpinBox.setMinimum(0.1)
        self.ob_search_range_doubleSpinBox.setSingleStep(0.5)        ###Max도 정하기!!!!!!!!!!!
        self.ob_search_range_doubleSpinBox.setObjectName("ob_search_range_doubleSpinBox")

        ### QLabel : Predict Step
        self.predict_step_label = QtWidgets.QLabel(A_star_Simulator)
        self.predict_step_label.setObjectName("predict_step_label")

        ### QSpinBox : Predict Step
        self.predict_step_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.predict_step_spinBox.setMinimum(1)
        self.predict_step_spinBox.setMaximum(10)
        self.predict_step_spinBox.setObjectName("predict_step_spinBox")

        ### QLabel : Predict Step Size
        self.predict_step_size_label = QtWidgets.QLabel(A_star_Simulator)
        self.predict_step_size_label.setObjectName("predict_step_size_label")

        ### QSpinBox : Predict Step Size
        self.predict_step_size_doubleSpinBox = QtWidgets.QDoubleSpinBox(A_star_Simulator)
        self.predict_step_size_doubleSpinBox.setMinimum(0.5)
        self.predict_step_size_doubleSpinBox.setSingleStep(0.5)        ###Max도 정하기!!!!!!!!!!!
        self.predict_step_size_doubleSpinBox.setObjectName("predict_step_size_doubleSpinBox")

        ### QLabel : g-value Rotate Gain(~45)
        self.g_value_rotate_gain_45_label = QtWidgets.QLabel(A_star_Simulator)
        self.g_value_rotate_gain_45_label.setObjectName("g_value_rotate_gain_45_label")

        ### QSDoublepinBox : g-value Rotate Gain(~45)
        self.g_value_rotate_gain_45_doubleSpinBox = QtWidgets.QDoubleSpinBox(A_star_Simulator)
        self.g_value_rotate_gain_45_doubleSpinBox.setMinimum(0)
        self.g_value_rotate_gain_45_doubleSpinBox.setSingleStep(0.1)  ###Max도 정하기!!!!!!!!!!!
        self.g_value_rotate_gain_45_doubleSpinBox.setObjectName("g_value_rotate_gain_45_doubleSpinBox")

        ### QLabel : g-value Rotate Gain(~90)
        self.g_value_rotate_gain_90_label = QtWidgets.QLabel(A_star_Simulator)
        self.g_value_rotate_gain_90_label.setObjectName("g_value_rotate_gain_90_label")

        ### QSDoublepinBox : g-value Rotate Gain(~90)
        self.g_value_rotate_gain_90_doubleSpinBox = QtWidgets.QDoubleSpinBox(A_star_Simulator)
        self.g_value_rotate_gain_90_doubleSpinBox.setMinimum(0)
        self.g_value_rotate_gain_90_doubleSpinBox.setSingleStep(0.1)  ###Max도 정하기!!!!!!!!!!!
        self.g_value_rotate_gain_90_doubleSpinBox.setObjectName("g_value_rotate_gain_90_doubleSpinBox")

        ### QLabel : g-value Rotate Gain(~180)
        self.g_value_rotate_gain_180_label = QtWidgets.QLabel(A_star_Simulator)
        self.g_value_rotate_gain_180_label.setObjectName("g_value_rotate_gain_180_label")

        ### QSDoublepinBox : g-value Rotate Gain(~180)
        self.g_value_rotate_gain_180_doubleSpinBox = QtWidgets.QDoubleSpinBox(A_star_Simulator)
        self.g_value_rotate_gain_180_doubleSpinBox.setMinimum(0)
        self.g_value_rotate_gain_180_doubleSpinBox.setSingleStep(0.1)  ###Max도 정하기!!!!!!!!!!!
        self.g_value_rotate_gain_180_doubleSpinBox.setObjectName("g_value_rotate_gain_180_doubleSpinBox")

        ### QLabel : h-value Gain
        self.h_value_gain_label = QtWidgets.QLabel(A_star_Simulator)
        self.h_value_gain_label.setObjectName("h_value_gain_label")

        ### QSDoublepinBox : h-value Gain
        self.h_value_gain_doubleSpinBox = QtWidgets.QDoubleSpinBox(A_star_Simulator)
        self.h_value_gain_doubleSpinBox.setMinimum(0)
        self.h_value_gain_doubleSpinBox.setSingleStep(0.1)  ###Max도 정하기!!!!!!!!!!!
        self.h_value_gain_doubleSpinBox.setObjectName("h_value_gain_doubleSpinBox")

        ### QLabel : Move Size
        self.move_size_label = QtWidgets.QLabel(A_star_Simulator)
        self.move_size_label.setObjectName("move_size_label")

        ### QSpinBox : Move Size
        self.move_size_doubleSpinBox = QtWidgets.QDoubleSpinBox(A_star_Simulator)
        self.move_size_doubleSpinBox.setMinimum(0.5)
        self.move_size_doubleSpinBox.setSingleStep(0.5)        ###Max도 정하기!!!!!!!!!!!
        self.move_size_doubleSpinBox.setObjectName("move_size_doubleSpinBox")

        ### QLabel : Arrival Range
        self.arrival_range_label = QtWidgets.QLabel(A_star_Simulator)
        self.arrival_range_label.setObjectName("arrival_range_label")

        ### QSpinBox : Arrival Range
        self.arrival_range_doubleSpinBox = QtWidgets.QDoubleSpinBox(A_star_Simulator)
        self.arrival_range_doubleSpinBox.setMinimum(0.5)
        self.arrival_range_doubleSpinBox.setSingleStep(0.5)        ###Max도 정하기!!!!!!!!!!!
        self.arrival_range_doubleSpinBox.setObjectName("arrival_range_doubleSpinBox")

        ### QLabel : Board
        self.board_label = QtWidgets.QLabel(A_star_Simulator)
        # self.board_label.setFont(QtGui.QFont(pointSize=16, weight=75))  # bold 어떻게 설정?
        self.board_label.setObjectName("board_label")

        ### QLabel : Status Label
        self.status_label = QtWidgets.QLabel(A_star_Simulator)
        self.status_label.setObjectName("status_label")

        ### QLineEdit : Status Text
        self.status_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.status_lineEdit.setObjectName("status_lineEdit")

        ### QLabel : Current Position Label
        self.cur_pos_label = QtWidgets.QLabel(A_star_Simulator)
        self.cur_pos_label.setObjectName("cur_pos_label")

        ### QLineEdit : Current Position Text
        self.cur_pos_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.cur_pos_lineEdit.setObjectName("cur_pos_lineEdit")

        ### QLabel : Current Heading Label
        self.cur_heading_label = QtWidgets.QLabel(A_star_Simulator)
        self.cur_heading_label.setObjectName("cur_heading_label")

        ### QLineEdit : Current Heading Text
        self.cur_heading_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.cur_heading_lineEdit.setObjectName("cur_heading_lineEdit")

        ### QLabel : Search Center Label
        self.search_center_label = QtWidgets.QLabel(A_star_Simulator)
        self.search_center_label.setObjectName("search_center_label")

        ### QLineEdit : Search Center Text
        self.search_center_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.search_center_lineEdit.setObjectName("search_center_lineEdit")

        ### QLabel : Search Direction Point Label
        self.search_direc_point_label = QtWidgets.QLabel(A_star_Simulator)
        self.search_direc_point_label.setObjectName("search_direc_point_label")

        ### QLineEdit : Search Direction Point Text
        self.search_direc_point_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.search_direc_point_lineEdit.setObjectName("search_direc_point_lineEdit")

        ### QPushButton : Start Simulation
        self.start_pushButton = QtWidgets.QPushButton(A_star_Simulator)
        self.start_pushButton.setObjectName("start_pushButton")

        ### QPushButton : Pause/Restart Simulation
        self.pause_and_restart_pushButton = QtWidgets.QPushButton(A_star_Simulator)
        self.pause_and_restart_pushButton.setObjectName("pause_and_restart_pushButton")

        ### QPushButton : Reset
        self.reset_pushButton = QtWidgets.QPushButton(A_star_Simulator)
        self.reset_pushButton.setObjectName("reset_pushButton")

        ################ Layout
        ### Main Layout
        layout = QtWidgets.QGridLayout()
        A_star_Simulator.setLayout(layout)

        ### Settings Layout
        settings_groupBox = QtWidgets.QGroupBox('Settings') #라벨 삭제?
        settings_grid = QtWidgets.QGridLayout()

        settings_grid.addWidget(self.settings_label, 0, 0, 2, 4)

        settings_grid.addWidget(self.start_point_label, 2, 0, 1, 2)
        settings_grid.addWidget(self.start_point_x_spinBox, 2, 2)
        settings_grid.addWidget(self.start_point_y_spinBox, 2, 3)

        settings_grid.addWidget(self.start_heading_label, 3, 0, 1, 2)
        settings_grid.addWidget(self.start_heading_x_spinBox, 3, 2)
        settings_grid.addWidget(self.start_heading_y_spinBox, 3, 3)

        settings_grid.addWidget(self.end_point_label, 4, 0, 1, 2)
        settings_grid.addWidget(self.end_point_x_spinBox, 4, 2)
        settings_grid.addWidget(self.end_point_y_spinBox, 4, 3)

        settings_grid.addWidget(self.ob_search_range_label, 5, 0, 1, 2)
        settings_grid.addWidget(self.ob_search_range_doubleSpinBox, 5, 2, 1, 2)

        settings_grid.addWidget(self.predict_step_label, 6, 0, 1, 2)
        settings_grid.addWidget(self.predict_step_spinBox, 6, 2, 1, 2)

        settings_grid.addWidget(self.predict_step_size_label, 7, 0, 1, 2)
        settings_grid.addWidget(self.predict_step_size_doubleSpinBox, 7, 2, 1, 2)

        settings_grid.addWidget(self.g_value_rotate_gain_45_label, 8, 0, 1, 2)
        settings_grid.addWidget(self.g_value_rotate_gain_45_doubleSpinBox, 8, 2, 1, 2)

        settings_grid.addWidget(self.g_value_rotate_gain_90_label, 9, 0, 1, 2)
        settings_grid.addWidget(self.g_value_rotate_gain_90_doubleSpinBox, 9, 2, 1, 2)

        settings_grid.addWidget(self.g_value_rotate_gain_180_label, 10, 0, 1, 2)
        settings_grid.addWidget(self.g_value_rotate_gain_180_doubleSpinBox, 10, 2, 1, 2)

        settings_grid.addWidget(self.h_value_gain_label, 11, 0, 1, 2)
        settings_grid.addWidget(self.h_value_gain_doubleSpinBox, 11, 2, 1, 2)

        settings_grid.addWidget(self.move_size_label, 12, 0, 1, 2)
        settings_grid.addWidget(self.move_size_doubleSpinBox, 12, 2, 1, 2)

        settings_grid.addWidget(self.arrival_range_label, 13, 0, 1, 2)
        settings_grid.addWidget(self.arrival_range_doubleSpinBox, 13, 2, 1, 2)

        settings_groupBox.setLayout(settings_grid)

        ### Board Layout
        board_groupBox = QtWidgets.QGroupBox('Board')  # 라벨 삭제?
        board_grid = QtWidgets.QGridLayout()

        board_grid.addWidget(self.board_label, 0, 0, 2, 4)

        board_grid.addWidget(self.status_label, 2, 0, 1, 2)
        board_grid.addWidget(self.status_lineEdit, 2, 2, 1, 2)

        board_grid.addWidget(self.cur_pos_label, 3, 0, 1, 2)
        board_grid.addWidget(self.cur_pos_lineEdit, 3, 2, 1, 2)

        board_grid.addWidget(self.cur_heading_label, 4, 0, 1, 2)
        board_grid.addWidget(self.cur_heading_lineEdit, 4, 2, 1, 2)

        board_grid.addWidget(self.search_center_label, 5, 0, 1, 2)
        board_grid.addWidget(self.search_center_lineEdit, 5, 2, 1, 2)

        board_grid.addWidget(self.search_direc_point_label, 6, 0, 1, 2)
        board_grid.addWidget(self.search_direc_point_lineEdit, 6, 2, 1, 2)

        board_groupBox.setLayout(board_grid)

        ### Main Layout
        layout.addWidget(self.graphicsView, 0, 0, 21, 15) #숫자 검토

        layout.addWidget(self.start_pushButton, 21, 0, 1, 5)
        layout.addWidget(self.pause_and_restart_pushButton, 21, 5, 1, 5)
        layout.addWidget(self.reset_pushButton, 21, 10, 1, 5)

        layout.addWidget(settings_groupBox, 0, 15, 14, 4)
        layout.addWidget(board_groupBox, 15, 15, 7, 4)

        ################ Initial Setting
        self.retranslate_UI(A_star_Simulator)

        ################ Button Click Event & Connection
        self.start_pushButton.clicked.connect(A_star_Simulator.start)
        self.pause_and_restart_pushButton.clicked.connect(A_star_Simulator.pause)
        self.reset_pushButton.clicked.connect(A_star_Simulator.reset)
        QtCore.QMetaObject.connectSlotsByName(A_star_Simulator)

    def retranslate_UI(self, A_star_Simulator):
        _translate = QtCore.QCoreApplication.translate
        A_star_Simulator.setWindowTitle(_translate("A_star_Simulator", "A star Simulator"))

        self.start_pushButton.setText(_translate("A_star_Simulator", "Start Simulation"))
        self.pause_and_restart_pushButton.setText(_translate("A_star_Simulator", "Pause"))
        self.reset_pushButton.setText(_translate("A_star_Simulator", "Reset"))

        self.settings_label.setText(_translate("A_star_Simulator", "Settings"))
        self.start_point_label.setText(_translate("A_star_Simulator", "Start Point"))
        self.start_heading_label.setText(_translate("A_star_Simulator", "Start Heading"))
        self.end_point_label.setText(_translate("A_star_Simulator", "End Point"))
        self.ob_search_range_label.setText(_translate("A_star_Simulator", "Obstacle Search Range"))
        self.predict_step_label.setText(_translate("A_star_Simulator", "Predict Step"))
        self.predict_step_size_label.setText(_translate("A_star_Simulator", "Predict Step Size"))
        self.g_value_rotate_gain_45_label.setText(_translate("A_star_Simulator", "g: 45 Rotate Gain"))
        self.g_value_rotate_gain_90_label.setText(_translate("A_star_Simulator", "   90 Rotate Gain"))
        self.g_value_rotate_gain_180_label.setText(_translate("A_star_Simulator", "  180 Rotate Gain"))
        self.h_value_gain_label.setText(_translate("A_star_Simulator", "h: Goal Cost Gain"))
        self.move_size_label.setText(_translate("A_star_Simulator", "Move Size"))
        self.arrival_range_label.setText(_translate("A_star_Simulator", "Arrival Check Range"))
        self.board_label.setText(_translate("A_star_Simulator", "Board"))
        self.status_label.setText(_translate("A_star_Simulator", "Status"))
        self.cur_pos_label.setText(_translate("A_star_Simulator", "Current Position"))
        self.cur_heading_label.setText(_translate("A_star_Simulator", "Current Heading"))
        self.search_center_label.setText(_translate("A_star_Simulator", "Search Center"))
        self.search_direc_point_label.setText(_translate("A_star_Simulator", "Search Direction Points"))
