from PyQt5 import QtGui
from PyQt5 import QtCore
from PyQt5 import QtWidgets


class UI_Simulator(object):
    def set_UI(self, A_star_Simulator):
        A_star_Simulator.setObjectName("A_star_Simulator")
        A_star_Simulator.resize(1012, 670) #수정하기

        ### QGraphicView : 시뮬레이터 화면
        self.graphicsView = QtWidgets.QGraphicsView(A_star_Simulator)
        # self.graphicsView.setGeometry(QtCore.QRect(20, 50, 600, 600)) #수정하기
        self.graphicsView.setObjectName("graphicsView") #수정하기

        ### QLabel : Input Value
        self.input_label = QtWidgets.QLabel(A_star_Simulator)
        # self.label.setGeometry(QtCore.QRect(630, 580, 261, 21))
        # font = QtGui.QFont()
        # font.setPointSize(16)
        # font.setBold(True)
        # font.setWeight(75)
        # self.label.setFont(font)
        self.input_label.setObjectName("input_label")

        ### QLabel : Start Point
        self.start_point_label = QtWidgets.QLabel(A_star_Simulator)
        # self.label.setGeometry(QtCore.QRect(630, 580, 261, 21))
        # font = QtGui.QFont()
        # font.setPointSize(16)
        # font.setBold(True)
        # font.setWeight(75)
        # self.label.setFont(font)
        self.start_point_label.setObjectName("start_point_label")

        ### QSpinBox : Start Point(X)
        self.start_point_x_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        # self.Possible_passes_spinBox.setGeometry(QtCore.QRect(940, 580, 60, 25))
        # font = QtGui.QFont()
        # font.setPointSize(16)
        # font.setBold(True)
        # font.setWeight(75)
        # self.Possible_passes_spinBox.setFont(font)
        self.start_point_x_spinBox.setMinimum(0)
        ###Max도 정하기!!!!!!!!!!!
        self.start_point_x_spinBox.setObjectName("start_point_x_spinBox")

        ### QSpinBox : Start Point(Y)
        self.start_point_y_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.start_point_y_spinBox.setMinimum(0)
        ###Max도 정하기!!!!!!!!!!!
        self.start_point_y_spinBox.setObjectName("start_point_y_spinBox")


        ### QLabel : Start Heading
        self.start_heading_label = QtWidgets.QLabel(A_star_Simulator)
        self.start_heading_label.setObjectName("start_heading_label")

        ### QSpinBox : Start Heading(X)
        self.start_heading_x_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.start_heading_x_spinBox.setMinimum(0)
        ###Max도 정하기!!!!!!!!!!!
        self.start_heading_x_spinBox.setObjectName("start_heading_x_spinBox")

        ### QSpinBox : Start Point(Y)
        self.start_heading_y_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.start_heading_y_spinBox.setMinimum(0)
        ###Max도 정하기!!!!!!!!!!!
        self.start_heading_y_spinBox.setObjectName("start_heading_y_spinBox")


        ### QLabel : End Point
        self.end_point_label = QtWidgets.QLabel(A_star_Simulator)
        self.end_point_label.setObjectName("end_point_label")

        ### QSpinBox : End Point(X)
        self.end_point_x_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.end_point_x_spinBox.setMinimum(0)
        ###Max도 정하기!!!!!!!!!!!
        self.end_point_x_spinBox.setObjectName("end_point_x_spinBox")

        ### QSpinBox : End Point(Y)
        self.end_point_y_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.end_point_y_spinBox.setMinimum(0)
        ###Max도 정하기!!!!!!!!!!!
        self.end_point_y_spinBox.setObjectName("end_point_y_spinBox")


        ### QLabel : Obstacle Search Range
        self.ob_search_range_label = QtWidgets.QLabel(A_star_Simulator)
        self.ob_search_range_label.setObjectName("ob_search_range_label")

        ### QSpinBox : Obstacle Search Range
        self.ob_search_range_doubleSpinBox = QtWidgets.QDoubleSpinBox(A_star_Simulator)
        self.ob_search_range_doubleSpinBox.setMinimum(0)
        self.ob_search_range_doubleSpinBox.setSingleStep(0.5)
        ###Max도 정하기!!!!!!!!!!!
        self.ob_search_range_doubleSpinBox.setObjectName("ob_search_range_doubleSpinBox")


        ### QLabel : Predict Step
        self.predict_step_label = QtWidgets.QLabel(A_star_Simulator)
        self.predict_step_label.setObjectName("predict_step_label")

        ### QSpinBox : Predict Step
        self.predict_step_spinBox = QtWidgets.QSpinBox(A_star_Simulator)
        self.predict_step_spinBox.setMinimum(1)
        self.predict_step_spinBox.setMinimum(10)
        self.predict_step_spinBox.setObjectName("predict_step_spinBox")


        ### QLabel : Predict Step Size
        self.predict_step_size_label = QtWidgets.QLabel(A_star_Simulator)
        self.predict_step_size_label.setObjectName("predict_step_size_label")

        ### QSpinBox : Predict Step Size
        self.predict_step_size_doubleSpinBox = QtWidgets.QDoubleSpinBox(A_star_Simulator)
        self.predict_step_size_doubleSpinBox.setMinimum(0.5)
        self.predict_step_size_doubleSpinBox.setSingleStep(0.5)
        ###Max도 정하기!!!!!!!!!!!
        self.predict_step_size_doubleSpinBox.setObjectName("predict_step_size_doubleSpinBox")


        ### QLabel : Move Size
        self.move_size_label = QtWidgets.QLabel(A_star_Simulator)
        self.move_size_label.setObjectName("move_size_label")

        ### QSpinBox : Move Size
        self.move_size_doubleSpinBox = QtWidgets.QDoubleSpinBox(A_star_Simulator)
        self.move_size_doubleSpinBox.setMinimum(0.5)
        self.move_size_doubleSpinBox.setSingleStep(0.5)
        ###Max도 정하기!!!!!!!!!!!
        self.move_size_doubleSpinBox.setObjectName("move_size_doubleSpinBox")


        ### QLabel : Arrival Range
        self.arrival_range_label = QtWidgets.QLabel(A_star_Simulator)
        self.arrival_range_label.setObjectName("arrival_range_label")

        ### QSpinBox : Arrival Range
        self.arrival_range_doubleSpinBox = QtWidgets.QDoubleSpinBox(A_star_Simulator)
        self.arrival_range_doubleSpinBox.setMinimum(0.5) ###확인!!!
        self.arrival_range_doubleSpinBox.setSingleStep(0.5)
        ###Max도 정하기!!!!!!!!!!!
        self.arrival_range_doubleSpinBox.setObjectName("arrival_range_doubleSpinBox")


        ### QLabel : Status
        self.status_label = QtWidgets.QLabel(A_star_Simulator)
        self.status_label.setObjectName("status_label")

        ### QLineEdit : Status
        self.status_lineEdit = QtWidgets.QLineEdit(A_star_Simulator)
        #관련 설정할 것???
        self.status_lineEdit.setObjectName("status_lineEdit")


        ### QPushButton : Start Simulation
        self.start_pushButton = QtWidgets.QPushButton(A_star_Simulator)
        # self.Start_pushButton.setGeometry(QtCore.QRect(630, 620, 170, 30))
        # font = QtGui.QFont()
        # font.setPointSize(16)
        # font.setBold(True)
        # font.setWeight(75)
        # self.Start_pushButton.setFont(font)
        self.start_pushButton.setObjectName("start_pushButton")

        ### QPushButton : Pause Simulation
        self.pause_pushButton = QtWidgets.QPushButton(A_star_Simulator)
        self.pause_pushButton.setObjectName("pause_pushButton")

        ### QPushButton : Reset
        self.reset_pushButton = QtWidgets.QPushButton(A_star_Simulator)
        self.reset_pushButton.setObjectName("reset_pushButton")


        ## Layout
        layout = QtWidgets.QGridLayout()

        layout.addWidget(self.graphicsView, 0, 0, 10, 10)

        #그룹박스로 묶기
        layout.addWidget(self.input_label, 0, 1)

        layout.addWidget(self.start_point_label, 1, 1, 1, 2)
        layout.addWidget(self.start_point_x_spinBox, 1, 2)
        layout.addWidget(self.start_point_y_spinBox, 1, 3)

        layout.addWidget(self.start_heading_label, 2, 1, 1, 2)
        layout.addWidget(self.start_heading_x_spinBox, 2, 2)
        layout.addWidget(self.start_heading_y_spinBox, 2, 3)

        layout.addWidget(self.end_point_label, 3, 1, 1, 2)
        layout.addWidget(self.end_point_x_spinBox, 3, 2)
        layout.addWidget(self.end_point_y_spinBox, 3, 3)

        layout.addWidget(self.ob_search_range_label, 4, 1, 1, 2)
        layout.addWidget(self.ob_search_range_doubleSpinBox, 4, 2, 1, 2)

        layout.addWidget(self.predict_step_label, 5, 1, 1, 2)
        layout.addWidget(self.predict_step_spinBox, 5, 2, 1, 2)

        layout.addWidget(self.predict_step_size_label, 6, 1, 1, 2)
        layout.addWidget(self.predict_step_size_doubleSpinBox, 6, 2, 1, 2)

        layout.addWidget(self.move_size_label, 7, 1, 1, 2)
        layout.addWidget(self.move_size_doubleSpinBox, 7, 2, 1, 2)

        layout.addWidget(self.arrival_range_label, 8, 1, 1, 2)
        layout.addWidget(self.arrival_range_doubleSpinBox, 8, 2, 1, 2)

        layout.addWidget(self.status_label, 9, 1, 1, 2)
        layout.addWidget(self.status_lineEdit, 9, 2, 1, 2)

        layout.addWidget(self.start_pushButton, 10, 0, 1, 10)
        layout.addWidget(self.pause_pushButton, 10, 1, 1, 2)
        layout.addWidget(self.reset_pushButton, 10, 3, 1, 2)

        self.retranslate_UI(A_star_Simulator)
        self.start_pushButton.clicked.connect(A_star_Simulator.start)
        self.pause_pushButton.clicked.connect(A_star_Simulator.pause)
        self.reset_pushButton.clicked.connect(A_star_Simulator.reset)
        QtCore.QMetaObject.connectSlotsByName(A_star_Simulator)

    def retranslate_UI(self, A_star_Simulator):
        _translate = QtCore.QCoreApplication.translate
        A_star_Simulator.setWindowTitle(_translate("A_star_Simulator", "A star Simulator"))

        self.start_pushButton.setText(_translate("A_star_Simulator", "Start Simulation"))
        self.pause_pushButton.setText(_translate("A_star_Simulator", "Pause"))
        self.reset_pushButton.setText(_translate("A_star_Simulator", "Reset"))

        self.input_label.setText(_translate("A_star_Simulator", "Input Values Control"))
        self.start_point_label.setText(_translate("A_star_Simulator", "Start Point"))
        self.start_heading_label.setText(_translate("A_star_Simulator", "Start Heading"))
        self.end_point_label.setText(_translate("A_star_Simulator", "End Point"))
        self.ob_search_range_label.setText(_translate("A_star_Simulator", "Obstacle Search Range"))
        self.predict_step_label.setText(_translate("A_star_Simulator", "Predict Step"))
        self.predict_step_size_label.setText(_translate("A_star_Simulator", "Predict Step Size"))
        self.move_size_label.setText(_translate("A_star_Simulator", "Move Size"))
        self.arrival_range_label.setText(_translate("A_star_Simulator", "Arrival Check Range"))
        self.status_label.setText(_translate("A_star_Simulator", "Now Status"))

        # self.status_lineEdit.setText(_translate("A_star_Simulator", "Now Status"))
