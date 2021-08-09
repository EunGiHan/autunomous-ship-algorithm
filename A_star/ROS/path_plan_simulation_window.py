#!/usr/bin/env python

from PyQt5 import QtGui
from PyQt5 import QtCore
from PyQt5 import QtWidgets


class UI_Simulator(object):
    def set_UI(self, A_star_Simulator):
        ################ Simulator
        A_star_Simulator.setObjectName("A_star_Simulator")
        A_star_Simulator.resize(1100, 750)

        ################ Widgets
        ### QGraphicView
        self.graphicsView = QtWidgets.QGraphicsView(A_star_Simulator)
        # self.graphicsView.setGeometry(QtCore.QRect(0, 0, 600, 600))
        self.graphicsView.setObjectName("graphicsView") 

        ### QLabel : Settings Value
        self.settings_label = QtWidgets.QLabel(A_star_Simulator)
        self.settings_label.setObjectName("settings_label")

        ### QLabel : Obstacle Search Range
        self.ob_search_range_label = QtWidgets.QLabel(A_star_Simulator)
        self.ob_search_range_label.setObjectName("ob_search_range_label")

        ### QLineEdit : Obstacle Search Range
        self.ob_search_range_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.ob_search_range_lineEdit.setObjectName("ob_search_range_lineEdit")

        ### QLabel : Predict Step
        self.predict_step_label = QtWidgets.QLabel(A_star_Simulator)
        self.predict_step_label.setObjectName("predict_step_label")

        ### QSpinBox : Predict Step
        self.predict_step_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.predict_step_lineEdit.setObjectName("predict_step_lineEdit")

        ### QLabel : Predict Step Size
        self.predict_step_size_label = QtWidgets.QLabel(A_star_Simulator)
        self.predict_step_size_label.setObjectName("predict_step_size_label")

        ### QLineEdit : Predict Step Size
        self.predict_step_size_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.predict_step_size_lineEdit.setObjectName("predict_step_size_lineEdit")

        ### QLabel : g-value Rotate Gain(~45)
        self.g_value_rotate_gain_45_label = QtWidgets.QLabel(A_star_Simulator)
        self.g_value_rotate_gain_45_label.setObjectName("g_value_rotate_gain_45_label")

        ### QLineEdit : Predict Step Size
        self.g_value_rotate_gain_45_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.g_value_rotate_gain_45_lineEdit.setObjectName("g_value_rotate_gain_45_lineEdit")

        ### QLabel : g-value Rotate Gain(~90)
        self.g_value_rotate_gain_90_label = QtWidgets.QLabel(A_star_Simulator)
        self.g_value_rotate_gain_90_label.setObjectName("g_value_rotate_gain_90_label")

        ### QLineEdit : Predict Step Size
        self.g_value_rotate_gain_90_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.g_value_rotate_gain_90_lineEdit.setObjectName("g_value_rotate_gain_90_lineEdit")

        ### QLabel : g-value Rotate Gain(~180)
        self.g_value_rotate_gain_180_label = QtWidgets.QLabel(A_star_Simulator)
        self.g_value_rotate_gain_180_label.setObjectName("g_value_rotate_gain_180_label")

        ### QLineEdit : Predict Step Size
        self.g_value_rotate_gain_180_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.g_value_rotate_gain_180_lineEdit.setObjectName("g_value_rotate_gain_180_lineEdit")

        ### QLabel : h-value Gain
        self.h_value_gain_label = QtWidgets.QLabel(A_star_Simulator)
        self.h_value_gain_label.setObjectName("h_value_gain_label")

        ### QLineEdit : Predict Step Size
        self.h_value_gain_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.h_value_gain_lineEdit.setObjectName("h_value_gain_lineEdit")

        ### QLabel : Arrival Range
        self.arrival_range_label = QtWidgets.QLabel(A_star_Simulator)
        self.arrival_range_label.setObjectName("arrival_range_label")

        ### QLineEdit : Arrival Range
        self.arrival_range_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.arrival_range_lineEdit.setObjectName("arrival_range_lineEdit")

        ### QLabel : Board
        self.board_label = QtWidgets.QLabel(A_star_Simulator)
        self.board_label.setObjectName("board_label")

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

        ### QLabel : Next Goal
        self.next_goal_label = QtWidgets.QLabel(A_star_Simulator)
        self.next_goal_label.setObjectName("next_goal_label")

        ### QLineEdit : Next Goal
        self.next_goal_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        self.next_goal_lineEdit.setObjectName("next_goal_lineEdit")

        ################ Layout
        ### Main Layout
        layout = QtWidgets.QGridLayout()
        A_star_Simulator.setLayout(layout)

        ### Settings Layout
        settings_groupBox = QtWidgets.QGroupBox('Settings') 
        settings_grid = QtWidgets.QGridLayout()

        settings_grid.addWidget(self.settings_label, 0, 0, 2, 4)

        settings_grid.addWidget(self.ob_search_range_label, 1, 0, 1, 2)
        settings_grid.addWidget(self.ob_search_range_lineEdit, 1, 2, 1, 2)

        settings_grid.addWidget(self.predict_step_label, 2, 0, 1, 2)
        settings_grid.addWidget(self.predict_step_lineEdit, 2, 2, 1, 2)

        settings_grid.addWidget(self.predict_step_size_label, 3, 0, 1, 2)
        settings_grid.addWidget(self.predict_step_size_lineEdit, 3, 2, 1, 2)

        settings_grid.addWidget(self.g_value_rotate_gain_45_label, 4, 0, 1, 2)
        settings_grid.addWidget(self.g_value_rotate_gain_45_lineEdit, 4, 2, 1, 2)

        settings_grid.addWidget(self.g_value_rotate_gain_90_label, 5, 0, 1, 2)
        settings_grid.addWidget(self.g_value_rotate_gain_90_lineEdit, 5, 2, 1, 2)

        settings_grid.addWidget(self.g_value_rotate_gain_180_label, 6, 0, 1, 2)
        settings_grid.addWidget(self.g_value_rotate_gain_180_lineEdit, 6, 2, 1, 2)

        settings_grid.addWidget(self.h_value_gain_label, 7, 0, 1, 2)
        settings_grid.addWidget(self.h_value_gain_lineEdit, 7, 2, 1, 2)

        settings_grid.addWidget(self.arrival_range_label, 8, 0, 1, 2)
        settings_grid.addWidget(self.arrival_range_lineEdit, 8, 2, 1, 2)

        settings_groupBox.setLayout(settings_grid)

        ### Board Layout
        board_groupBox = QtWidgets.QGroupBox('Board') 
        board_grid = QtWidgets.QGridLayout()

        board_grid.addWidget(self.board_label, 0, 0, 2, 4)

        board_grid.addWidget(self.cur_pos_label, 1, 0, 1, 2)
        board_grid.addWidget(self.cur_pos_lineEdit, 1, 2, 1, 2)

        board_grid.addWidget(self.cur_heading_label, 2, 0, 1, 2)
        board_grid.addWidget(self.cur_heading_lineEdit, 2, 2, 1, 2)

        board_grid.addWidget(self.next_goal_label, 3, 0, 1, 2)
        board_grid.addWidget(self.next_goal_lineEdit, 3, 2, 1, 2)

        board_groupBox.setLayout(board_grid)

        ### Main Layout
        layout.addWidget(self.graphicsView, 0, 0, 15, 12)

        layout.addWidget(settings_groupBox, 0, 14, 9, 4)
        layout.addWidget(board_groupBox, 11, 14, 4, 4)

        ################ Initial Setting
        self.retranslate_UI(A_star_Simulator)

    def retranslate_UI(self, A_star_Simulator):
        _translate = QtCore.QCoreApplication.translate
        A_star_Simulator.setWindowTitle(_translate("A_star_Simulator", "A star Simulator"))

        self.settings_label.setText(_translate("A_star_Simulator", "Settings"))
        self.ob_search_range_label.setText(_translate("A_star_Simulator", "Obstacle Search Range"))
        self.predict_step_label.setText(_translate("A_star_Simulator", "Predict Step"))
        self.predict_step_size_label.setText(_translate("A_star_Simulator", "Predict Step Size"))
        self.g_value_rotate_gain_45_label.setText(_translate("A_star_Simulator", "g: 45 Rotate Gain"))
        self.g_value_rotate_gain_90_label.setText(_translate("A_star_Simulator", "   90 Rotate Gain"))
        self.g_value_rotate_gain_180_label.setText(_translate("A_star_Simulator", "  180 Rotate Gain"))
        self.h_value_gain_label.setText(_translate("A_star_Simulator", "h: Goal Cost Gain"))
        self.arrival_range_label.setText(_translate("A_star_Simulator", "Arrival Check Range"))
        self.board_label.setText(_translate("A_star_Simulator", "Board"))
        self.cur_pos_label.setText(_translate("A_star_Simulator", "Current Position"))
        self.cur_heading_label.setText(_translate("A_star_Simulator", "Current Heading"))
        self.next_goal_label.setText(_translate("A_star_Simulator", "Next Goal"))
