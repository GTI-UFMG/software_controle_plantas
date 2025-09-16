# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main_windowwXzQvn.ui'
##
## Created by: Qt User Interface Compiler version 6.9.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QAbstractSpinBox, QApplication, QCheckBox, QComboBox,
    QCommandLinkButton, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QHeaderView, QLCDNumber, QLabel,
    QLayout, QLineEdit, QMainWindow, QPlainTextEdit,
    QPushButton, QRadioButton, QSizePolicy, QSlider,
    QSpacerItem, QSpinBox, QTabWidget, QTableWidget,
    QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget)

from pyqtgraph import PlotWidget
import resources_rc

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(747, 611)
        MainWindow.setMinimumSize(QSize(640, 480))
        MainWindow.setStyleSheet(u"font: 10pt \"Ubuntu\";")
        MainWindow.setAnimated(True)
        MainWindow.setTabShape(QTabWidget.TabShape.Rounded)
        MainWindow.setDockNestingEnabled(False)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayout = QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.tabWidge_main = QTabWidget(self.centralwidget)
        self.tabWidge_main.setObjectName(u"tabWidge_main")
        self.tabWidge_main.setEnabled(True)
        self.tabWidge_main.setTabShape(QTabWidget.TabShape.Rounded)
        self.tab_general_configuration = QWidget()
        self.tab_general_configuration.setObjectName(u"tab_general_configuration")
        self.tab_general_configuration.setEnabled(True)
        self.horizontalLayout_9 = QHBoxLayout(self.tab_general_configuration)
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.verticalLayout_15 = QVBoxLayout()
        self.verticalLayout_15.setObjectName(u"verticalLayout_15")
        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.groupBox_4 = QGroupBox(self.tab_general_configuration)
        self.groupBox_4.setObjectName(u"groupBox_4")
        self.lineEdit_duration = QLineEdit(self.groupBox_4)
        self.lineEdit_duration.setObjectName(u"lineEdit_duration")
        self.lineEdit_duration.setEnabled(False)
        self.lineEdit_duration.setGeometry(QRect(20, 60, 91, 21))
        self.label_17 = QLabel(self.groupBox_4)
        self.label_17.setObjectName(u"label_17")
        self.label_17.setGeometry(QRect(20, 20, 191, 41))
        self.label_17.setStyleSheet(u"font: 600 9pt \"Segoe UI\";")
        self.label_17.setFrameShape(QFrame.Shape.NoFrame)
        self.label_17.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.label_17.setWordWrap(True)
        self.label_18 = QLabel(self.groupBox_4)
        self.label_18.setObjectName(u"label_18")
        self.label_18.setGeometry(QRect(20, 130, 151, 41))
        self.label_18.setStyleSheet(u"font: 600 9pt \"Segoe UI\";")
        self.label_18.setFrameShape(QFrame.Shape.NoFrame)
        self.label_18.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.label_18.setWordWrap(True)
        self.spinBox_sample_time = QSpinBox(self.groupBox_4)
        self.spinBox_sample_time.setObjectName(u"spinBox_sample_time")
        self.spinBox_sample_time.setEnabled(True)
        self.spinBox_sample_time.setGeometry(QRect(20, 160, 101, 23))
        self.spinBox_sample_time.setWrapping(False)
        self.spinBox_sample_time.setFrame(True)
        self.spinBox_sample_time.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.spinBox_sample_time.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.UpDownArrows)
        self.spinBox_sample_time.setKeyboardTracking(True)
        self.spinBox_sample_time.setProperty(u"showGroupSeparator", False)
        self.spinBox_sample_time.setMinimum(2)
        self.spinBox_sample_time.setMaximum(500)
        self.spinBox_sample_time.setSingleStep(2)
        self.checkBox_confirm_ts = QCheckBox(self.groupBox_4)
        self.checkBox_confirm_ts.setObjectName(u"checkBox_confirm_ts")
        self.checkBox_confirm_ts.setGeometry(QRect(20, 190, 191, 41))
        self.checkBox_confirm_ts.setChecked(True)

        self.horizontalLayout_6.addWidget(self.groupBox_4)

        self.groupBox_controlled_output = QGroupBox(self.tab_general_configuration)
        self.groupBox_controlled_output.setObjectName(u"groupBox_controlled_output")
        self.groupBox_controlled_output.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.groupBox_controlled_output.setCheckable(False)
        self.verticalLayoutWidget_2 = QWidget(self.groupBox_controlled_output)
        self.verticalLayoutWidget_2.setObjectName(u"verticalLayoutWidget_2")
        self.verticalLayoutWidget_2.setGeometry(QRect(10, 20, 201, 80))
        self.verticalLayout_8 = QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.verticalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.radioButton_position = QRadioButton(self.verticalLayoutWidget_2)
        self.radioButton_position.setObjectName(u"radioButton_position")

        self.verticalLayout_8.addWidget(self.radioButton_position)

        self.radioButton_speed = QRadioButton(self.verticalLayoutWidget_2)
        self.radioButton_speed.setObjectName(u"radioButton_speed")
        self.radioButton_speed.setChecked(True)

        self.verticalLayout_8.addWidget(self.radioButton_speed)


        self.horizontalLayout_6.addWidget(self.groupBox_controlled_output)


        self.verticalLayout_15.addLayout(self.horizontalLayout_6)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.groupBox_reference_input = QGroupBox(self.tab_general_configuration)
        self.groupBox_reference_input.setObjectName(u"groupBox_reference_input")
        self.verticalLayoutWidget_3 = QWidget(self.groupBox_reference_input)
        self.verticalLayoutWidget_3.setObjectName(u"verticalLayoutWidget_3")
        self.verticalLayoutWidget_3.setGeometry(QRect(10, 20, 160, 80))
        self.verticalLayout_9 = QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.verticalLayout_9.setContentsMargins(0, 0, 0, 0)
        self.radioButton_manual = QRadioButton(self.verticalLayoutWidget_3)
        self.radioButton_manual.setObjectName(u"radioButton_manual")
        self.radioButton_manual.setChecked(True)

        self.verticalLayout_9.addWidget(self.radioButton_manual)

        self.radioButton_prg = QRadioButton(self.verticalLayoutWidget_3)
        self.radioButton_prg.setObjectName(u"radioButton_prg")
        self.radioButton_prg.setChecked(False)

        self.verticalLayout_9.addWidget(self.radioButton_prg)


        self.horizontalLayout_7.addWidget(self.groupBox_reference_input)

        self.groupBox_input_prg = QGroupBox(self.tab_general_configuration)
        self.groupBox_input_prg.setObjectName(u"groupBox_input_prg")
        self.verticalLayout_6 = QVBoxLayout(self.groupBox_input_prg)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.label_11 = QLabel(self.groupBox_input_prg)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setStyleSheet(u"font: 600 9pt \"Segoe UI\";")
        self.label_11.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)

        self.verticalLayout_6.addWidget(self.label_11)

        self.tableWidget_prog_input = QTableWidget(self.groupBox_input_prg)
        if (self.tableWidget_prog_input.columnCount() < 2):
            self.tableWidget_prog_input.setColumnCount(2)
        if (self.tableWidget_prog_input.rowCount() < 15):
            self.tableWidget_prog_input.setRowCount(15)
        __qtablewidgetitem = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(0, 0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(0, 1, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(1, 0, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(1, 1, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(2, 0, __qtablewidgetitem4)
        __qtablewidgetitem5 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(2, 1, __qtablewidgetitem5)
        __qtablewidgetitem6 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(3, 0, __qtablewidgetitem6)
        __qtablewidgetitem7 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(3, 1, __qtablewidgetitem7)
        __qtablewidgetitem8 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(4, 0, __qtablewidgetitem8)
        __qtablewidgetitem9 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(4, 1, __qtablewidgetitem9)
        __qtablewidgetitem10 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(5, 0, __qtablewidgetitem10)
        __qtablewidgetitem11 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(5, 1, __qtablewidgetitem11)
        __qtablewidgetitem12 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(6, 0, __qtablewidgetitem12)
        __qtablewidgetitem13 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(6, 1, __qtablewidgetitem13)
        __qtablewidgetitem14 = QTableWidgetItem()
        self.tableWidget_prog_input.setItem(7, 0, __qtablewidgetitem14)
        self.tableWidget_prog_input.setObjectName(u"tableWidget_prog_input")
        self.tableWidget_prog_input.setEnabled(False)
        self.tableWidget_prog_input.setFrameShape(QFrame.Shape.NoFrame)
        self.tableWidget_prog_input.setFrameShadow(QFrame.Shadow.Plain)
        self.tableWidget_prog_input.setLineWidth(2)
        self.tableWidget_prog_input.setMidLineWidth(0)
        self.tableWidget_prog_input.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.tableWidget_prog_input.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.tableWidget_prog_input.setGridStyle(Qt.PenStyle.SolidLine)
        self.tableWidget_prog_input.setSortingEnabled(True)
        self.tableWidget_prog_input.setRowCount(15)
        self.tableWidget_prog_input.setColumnCount(2)
        self.tableWidget_prog_input.horizontalHeader().setVisible(False)
        self.tableWidget_prog_input.horizontalHeader().setCascadingSectionResizes(False)
        self.tableWidget_prog_input.horizontalHeader().setDefaultSectionSize(50)
        self.tableWidget_prog_input.verticalHeader().setDefaultSectionSize(24)

        self.verticalLayout_6.addWidget(self.tableWidget_prog_input)


        self.horizontalLayout_7.addWidget(self.groupBox_input_prg)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_7.addItem(self.horizontalSpacer_2)


        self.verticalLayout_15.addLayout(self.horizontalLayout_7)

        self.verticalLayout_15.setStretch(0, 1)
        self.verticalLayout_15.setStretch(1, 1)

        self.horizontalLayout_9.addLayout(self.verticalLayout_15)

        self.tabWidge_main.addTab(self.tab_general_configuration, "")
        self.tab_comm_serial = QWidget()
        self.tab_comm_serial.setObjectName(u"tab_comm_serial")
        self.verticalLayout = QVBoxLayout(self.tab_comm_serial)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.groupBox_3 = QGroupBox(self.tab_comm_serial)
        self.groupBox_3.setObjectName(u"groupBox_3")
        self.pushButton_test_comm = QPushButton(self.groupBox_3)
        self.pushButton_test_comm.setObjectName(u"pushButton_test_comm")
        self.pushButton_test_comm.setGeometry(QRect(40, 150, 141, 51))
        self.pushButton_test_comm.setStyleSheet(u"background-color: rgb(162, 253, 255);")
        self.pushButton_test_comm.setCheckable(False)
        self.pushButton_test_comm.setFlat(False)
        self.label_13 = QLabel(self.groupBox_3)
        self.label_13.setObjectName(u"label_13")
        self.label_13.setGeometry(QRect(21, 30, 64, 23))
        self.label_13.setStyleSheet(u"font: 600 9pt \"Segoe UI\";")
        self.label_13.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_14 = QLabel(self.groupBox_3)
        self.label_14.setObjectName(u"label_14")
        self.label_14.setGeometry(QRect(20, 60, 64, 41))
        self.label_14.setStyleSheet(u"font: 600 9pt \"Segoe UI\";")
        self.label_14.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_14.setWordWrap(True)
        self.comboBox = QComboBox(self.groupBox_3)
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.setObjectName(u"comboBox")
        self.comboBox.setEnabled(False)
        self.comboBox.setGeometry(QRect(90, 70, 131, 24))
        self.comboBox.setMaxVisibleItems(5)
        self.label_15 = QLabel(self.groupBox_3)
        self.label_15.setObjectName(u"label_15")
        self.label_15.setGeometry(QRect(30, 100, 331, 51))
        self.label_15.setWordWrap(True)
        self.lineEdit_serial_port = QLineEdit(self.groupBox_3)
        self.lineEdit_serial_port.setObjectName(u"lineEdit_serial_port")
        self.lineEdit_serial_port.setEnabled(False)
        self.lineEdit_serial_port.setGeometry(QRect(90, 30, 191, 21))
        self.checkBox_auto_serial_port = QCheckBox(self.groupBox_3)
        self.checkBox_auto_serial_port.setObjectName(u"checkBox_auto_serial_port")
        self.checkBox_auto_serial_port.setGeometry(QRect(300, 30, 101, 20))
        self.checkBox_auto_serial_port.setChecked(True)
        self.checkBox_auto_serial_port.setTristate(False)

        self.horizontalLayout_4.addWidget(self.groupBox_3)

        self.horizontalLayout_4.setStretch(0, 1)

        self.verticalLayout.addLayout(self.horizontalLayout_4)

        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.line = QFrame(self.tab_comm_serial)
        self.line.setObjectName(u"line")
        self.line.setFrameShape(QFrame.Shape.HLine)
        self.line.setFrameShadow(QFrame.Shadow.Sunken)

        self.verticalLayout_5.addWidget(self.line)

        self.label_16 = QLabel(self.tab_comm_serial)
        self.label_16.setObjectName(u"label_16")
        self.label_16.setStyleSheet(u"font: 600 9pt \"Segoe UI\";")
        self.label_16.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_5.addWidget(self.label_16)

        self.plainTextEdit_message_area = QPlainTextEdit(self.tab_comm_serial)
        self.plainTextEdit_message_area.setObjectName(u"plainTextEdit_message_area")
        self.plainTextEdit_message_area.setFrameShape(QFrame.Shape.Box)
        self.plainTextEdit_message_area.setLineWidth(1)
        self.plainTextEdit_message_area.setMidLineWidth(0)

        self.verticalLayout_5.addWidget(self.plainTextEdit_message_area)


        self.verticalLayout.addLayout(self.verticalLayout_5)

        self.verticalLayout.setStretch(0, 1)
        self.verticalLayout.setStretch(1, 1)
        self.tabWidge_main.addTab(self.tab_comm_serial, "")
        self.tab_controller = QWidget()
        self.tab_controller.setObjectName(u"tab_controller")
        self.tab_controller.setEnabled(True)
        self.verticalLayout_11 = QVBoxLayout(self.tab_controller)
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.tabWidget = QTabWidget(self.tab_controller)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tab_general_config = QWidget()
        self.tab_general_config.setObjectName(u"tab_general_config")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tab_general_config.sizePolicy().hasHeightForWidth())
        self.tab_general_config.setSizePolicy(sizePolicy)
        self.verticalLayout_16 = QVBoxLayout(self.tab_general_config)
        self.verticalLayout_16.setObjectName(u"verticalLayout_16")
        self.groupBox_controller_choice = QGroupBox(self.tab_general_config)
        self.groupBox_controller_choice.setObjectName(u"groupBox_controller_choice")
        self.horizontalLayout_8 = QHBoxLayout(self.groupBox_controller_choice)
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.verticalLayout_12 = QVBoxLayout()
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.verticalLayout_12.setContentsMargins(-1, -1, -1, 0)
        self.verticalSpacer_3 = QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)

        self.verticalLayout_12.addItem(self.verticalSpacer_3)

        self.radioButton_open_loop = QRadioButton(self.groupBox_controller_choice)
        self.radioButton_open_loop.setObjectName(u"radioButton_open_loop")
        self.radioButton_open_loop.setChecked(True)

        self.verticalLayout_12.addWidget(self.radioButton_open_loop)

        self.radioButton_PID_s = QRadioButton(self.groupBox_controller_choice)
        self.radioButton_PID_s.setObjectName(u"radioButton_PID_s")
        self.radioButton_PID_s.setEnabled(True)
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.radioButton_PID_s.sizePolicy().hasHeightForWidth())
        self.radioButton_PID_s.setSizePolicy(sizePolicy1)
        self.radioButton_PID_s.setChecked(False)

        self.verticalLayout_12.addWidget(self.radioButton_PID_s)

        self.radioButton_FT_s = QRadioButton(self.groupBox_controller_choice)
        self.radioButton_FT_s.setObjectName(u"radioButton_FT_s")

        self.verticalLayout_12.addWidget(self.radioButton_FT_s)

        self.radioButton_PID_discrete = QRadioButton(self.groupBox_controller_choice)
        self.radioButton_PID_discrete.setObjectName(u"radioButton_PID_discrete")
        self.radioButton_PID_discrete.setChecked(False)

        self.verticalLayout_12.addWidget(self.radioButton_PID_discrete)

        self.radioButton_FT_discrete = QRadioButton(self.groupBox_controller_choice)
        self.radioButton_FT_discrete.setObjectName(u"radioButton_FT_discrete")

        self.verticalLayout_12.addWidget(self.radioButton_FT_discrete)

        self.radioButton_generic = QRadioButton(self.groupBox_controller_choice)
        self.radioButton_generic.setObjectName(u"radioButton_generic")

        self.verticalLayout_12.addWidget(self.radioButton_generic)

        self.verticalSpacer_2 = QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_12.addItem(self.verticalSpacer_2)


        self.horizontalLayout_8.addLayout(self.verticalLayout_12)

        self.horizontalSpacer_3 = QSpacerItem(30, 20, QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_8.addItem(self.horizontalSpacer_3)

        self.verticalLayout_17 = QVBoxLayout()
        self.verticalLayout_17.setObjectName(u"verticalLayout_17")
        self.verticalLayout_17.setSizeConstraint(QLayout.SizeConstraint.SetDefaultConstraint)
        self.label_8 = QLabel(self.groupBox_controller_choice)
        self.label_8.setObjectName(u"label_8")

        self.verticalLayout_17.addWidget(self.label_8)

        self.textEdit_control_info = QTextEdit(self.groupBox_controller_choice)
        self.textEdit_control_info.setObjectName(u"textEdit_control_info")
        self.textEdit_control_info.setStyleSheet(u"background-color: rgb(255, 254, 237);")
        self.textEdit_control_info.setFrameShape(QFrame.Shape.Box)
        self.textEdit_control_info.setFrameShadow(QFrame.Shadow.Sunken)
        self.textEdit_control_info.setLineWidth(2)
        self.textEdit_control_info.setReadOnly(True)
        self.textEdit_control_info.setTextInteractionFlags(Qt.TextInteractionFlag.LinksAccessibleByMouse|Qt.TextInteractionFlag.TextSelectableByMouse)

        self.verticalLayout_17.addWidget(self.textEdit_control_info)


        self.horizontalLayout_8.addLayout(self.verticalLayout_17)


        self.verticalLayout_16.addWidget(self.groupBox_controller_choice)

        self.groupBox_deadzone_comp = QGroupBox(self.tab_general_config)
        self.groupBox_deadzone_comp.setObjectName(u"groupBox_deadzone_comp")
        self.groupBox_deadzone_comp.setEnabled(True)
        sizePolicy.setHeightForWidth(self.groupBox_deadzone_comp.sizePolicy().hasHeightForWidth())
        self.groupBox_deadzone_comp.setSizePolicy(sizePolicy)
        self.groupBox_deadzone_comp.setMinimumSize(QSize(0, 220))
        self.groupBox_deadzone_comp.setCheckable(True)
        self.groupBox_deadzone_comp.setChecked(True)
        self.label_12 = QLabel(self.groupBox_deadzone_comp)
        self.label_12.setObjectName(u"label_12")
        self.label_12.setEnabled(True)
        self.label_12.setGeometry(QRect(260, 10, 161, 201))
        self.label_12.setPixmap(QPixmap(u":/images/GUI_resources/comp_dead-zone.png"))
        self.label_12.setScaledContents(False)
        self.layoutWidget = QWidget(self.groupBox_deadzone_comp)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(51, 80, 114, 48))
        self.gridLayout_2 = QGridLayout(self.layoutWidget)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.label_19 = QLabel(self.layoutWidget)
        self.label_19.setObjectName(u"label_19")
        self.label_19.setEnabled(True)

        self.gridLayout_2.addWidget(self.label_19, 0, 0, 1, 1)

        self.spinBox_Cp = QSpinBox(self.layoutWidget)
        self.spinBox_Cp.setObjectName(u"spinBox_Cp")
        self.spinBox_Cp.setEnabled(True)
        self.spinBox_Cp.setMaximum(255)
        self.spinBox_Cp.setValue(144)

        self.gridLayout_2.addWidget(self.spinBox_Cp, 0, 1, 1, 1)

        self.label_20 = QLabel(self.layoutWidget)
        self.label_20.setObjectName(u"label_20")
        self.label_20.setEnabled(True)

        self.gridLayout_2.addWidget(self.label_20, 1, 0, 1, 1)

        self.spinBox_Cn = QSpinBox(self.layoutWidget)
        self.spinBox_Cn.setObjectName(u"spinBox_Cn")
        self.spinBox_Cn.setEnabled(True)
        self.spinBox_Cn.setMinimum(-255)
        self.spinBox_Cn.setMaximum(0)
        self.spinBox_Cn.setValue(-139)

        self.gridLayout_2.addWidget(self.spinBox_Cn, 1, 1, 1, 1)

        self.label_26 = QLabel(self.groupBox_deadzone_comp)
        self.label_26.setObjectName(u"label_26")
        self.label_26.setGeometry(QRect(450, 30, 191, 151))
        self.label_26.setTextFormat(Qt.TextFormat.AutoText)
        self.label_26.setWordWrap(True)

        self.verticalLayout_16.addWidget(self.groupBox_deadzone_comp)

        self.verticalLayout_16.setStretch(0, 3)
        self.verticalLayout_16.setStretch(1, 2)
        self.tabWidget.addTab(self.tab_general_config, "")
        self.tab_PID_controller = QWidget()
        self.tab_PID_controller.setObjectName(u"tab_PID_controller")
        self.label_21 = QLabel(self.tab_PID_controller)
        self.label_21.setObjectName(u"label_21")
        self.label_21.setGeometry(QRect(20, 20, 321, 81))
        self.label_21.setFrameShape(QFrame.Shape.Box)
        self.label_21.setFrameShadow(QFrame.Shadow.Sunken)
        self.label_21.setPixmap(QPixmap(u":/images/GUI_resources/PID_in_s.png"))
        self.label_21.setScaledContents(False)
        self.groupBox_ct_ti = QGroupBox(self.tab_PID_controller)
        self.groupBox_ct_ti.setObjectName(u"groupBox_ct_ti")
        self.groupBox_ct_ti.setGeometry(QRect(50, 250, 281, 91))
        self.groupBox_ct_ti.setCheckable(True)
        self.label_23 = QLabel(self.groupBox_ct_ti)
        self.label_23.setObjectName(u"label_23")
        self.label_23.setGeometry(QRect(10, 40, 91, 16))
        self.lineEdit_ct_ti = QLineEdit(self.groupBox_ct_ti)
        self.lineEdit_ct_ti.setObjectName(u"lineEdit_ct_ti")
        self.lineEdit_ct_ti.setGeometry(QRect(100, 40, 151, 21))
        self.groupBox_ct_td = QGroupBox(self.tab_PID_controller)
        self.groupBox_ct_td.setObjectName(u"groupBox_ct_td")
        self.groupBox_ct_td.setGeometry(QRect(50, 370, 281, 91))
        self.groupBox_ct_td.setCheckable(True)
        self.groupBox_ct_td.setChecked(False)
        self.label_25 = QLabel(self.groupBox_ct_td)
        self.label_25.setObjectName(u"label_25")
        self.label_25.setGeometry(QRect(10, 40, 91, 16))
        self.lineEdit_ct_td = QLineEdit(self.groupBox_ct_td)
        self.lineEdit_ct_td.setObjectName(u"lineEdit_ct_td")
        self.lineEdit_ct_td.setGeometry(QRect(100, 40, 151, 21))
        self.groupBox_7 = QGroupBox(self.tab_PID_controller)
        self.groupBox_7.setObjectName(u"groupBox_7")
        self.groupBox_7.setGeometry(QRect(50, 130, 371, 91))
        self.groupBox_7.setCheckable(False)
        self.label_24 = QLabel(self.groupBox_7)
        self.label_24.setObjectName(u"label_24")
        self.label_24.setGeometry(QRect(10, 20, 31, 21))
        self.lineEdit_ct_kp = QLineEdit(self.groupBox_7)
        self.lineEdit_ct_kp.setObjectName(u"lineEdit_ct_kp")
        self.lineEdit_ct_kp.setGeometry(QRect(40, 20, 121, 21))
        self.label_22 = QLabel(self.groupBox_7)
        self.label_22.setObjectName(u"label_22")
        self.label_22.setGeometry(QRect(10, 40, 351, 41))
        self.tabWidget.addTab(self.tab_PID_controller, "")
        self.tab_FT_controller = QWidget()
        self.tab_FT_controller.setObjectName(u"tab_FT_controller")
        self.tabWidget.addTab(self.tab_FT_controller, "")
        self.tab_generic_controller = QWidget()
        self.tab_generic_controller.setObjectName(u"tab_generic_controller")
        self.verticalLayout_14 = QVBoxLayout(self.tab_generic_controller)
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.verticalLayout_13 = QVBoxLayout()
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.label_3 = QLabel(self.tab_generic_controller)
        self.label_3.setObjectName(u"label_3")

        self.verticalLayout_13.addWidget(self.label_3)

        self.plainTextEdit_ctrl_code = QPlainTextEdit(self.tab_generic_controller)
        self.plainTextEdit_ctrl_code.setObjectName(u"plainTextEdit_ctrl_code")
        font = QFont()
        font.setFamilies([u"Ubuntu"])
        font.setPointSize(12)
        font.setWeight(QFont.Medium)
        font.setItalic(False)
        font.setKerning(True)
        self.plainTextEdit_ctrl_code.setFont(font)
        self.plainTextEdit_ctrl_code.viewport().setProperty(u"cursor", QCursor(Qt.CursorShape.PointingHandCursor))
        self.plainTextEdit_ctrl_code.setStyleSheet(u"font: 500 12pt \"Ubuntu\";\n"
"background-color: rgb(237, 251, 255);")
        self.plainTextEdit_ctrl_code.setFrameShape(QFrame.Shape.WinPanel)
        self.plainTextEdit_ctrl_code.setFrameShadow(QFrame.Shadow.Plain)
        self.plainTextEdit_ctrl_code.setLineWidth(1)
        self.plainTextEdit_ctrl_code.setMidLineWidth(0)

        self.verticalLayout_13.addWidget(self.plainTextEdit_ctrl_code)

        self.verticalSpacer = QSpacerItem(818, 68, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)

        self.verticalLayout_13.addItem(self.verticalSpacer)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.verticalLayout_10 = QVBoxLayout()
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.label_4 = QLabel(self.tab_generic_controller)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setWordWrap(True)

        self.verticalLayout_10.addWidget(self.label_4)

        self.comboBox_ctrl_code = QComboBox(self.tab_generic_controller)
        self.comboBox_ctrl_code.addItem("")
        self.comboBox_ctrl_code.addItem("")
        self.comboBox_ctrl_code.addItem("")
        self.comboBox_ctrl_code.setObjectName(u"comboBox_ctrl_code")
        self.comboBox_ctrl_code.setMaxVisibleItems(8)
        self.comboBox_ctrl_code.setFrame(True)
        self.comboBox_ctrl_code.setModelColumn(0)

        self.verticalLayout_10.addWidget(self.comboBox_ctrl_code)

        self.pushButton_ctrl_code = QPushButton(self.tab_generic_controller)
        self.pushButton_ctrl_code.setObjectName(u"pushButton_ctrl_code")
        self.pushButton_ctrl_code.setStyleSheet(u"background-color: rgb(106, 230, 255);")
        self.pushButton_ctrl_code.setFlat(False)

        self.verticalLayout_10.addWidget(self.pushButton_ctrl_code)


        self.horizontalLayout_5.addLayout(self.verticalLayout_10)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_5.addItem(self.horizontalSpacer)

        self.pushButton_send_code_ESP32 = QPushButton(self.tab_generic_controller)
        self.pushButton_send_code_ESP32.setObjectName(u"pushButton_send_code_ESP32")
        self.pushButton_send_code_ESP32.setStyleSheet(u"QPushButton:hover {\n"
"   color: red;\n"
"	font: 700 9pt \"Segoe UI\";\n"
"}\n"
"QPushButton {\n"
"   background-color: rgb(255, 215, 215);\n"
"}")
        self.pushButton_send_code_ESP32.setFlat(False)

        self.horizontalLayout_5.addWidget(self.pushButton_send_code_ESP32)


        self.verticalLayout_13.addLayout(self.horizontalLayout_5)


        self.verticalLayout_14.addLayout(self.verticalLayout_13)

        self.tabWidget.addTab(self.tab_generic_controller, "")

        self.verticalLayout_11.addWidget(self.tabWidget)

        self.tabWidge_main.addTab(self.tab_controller, "")
        self.tab_experiment = QWidget()
        self.tab_experiment.setObjectName(u"tab_experiment")
        self.verticalLayout_4 = QVBoxLayout(self.tab_experiment)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.groupBox_time_vis = QGroupBox(self.tab_experiment)
        self.groupBox_time_vis.setObjectName(u"groupBox_time_vis")
        self.groupBox_time_vis.setEnabled(True)
        self.label_2 = QLabel(self.groupBox_time_vis)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(10, 140, 61, 51))
        self.label_2.setScaledContents(False)
        self.label_2.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_2.setWordWrap(True)
        self.label_6 = QLabel(self.groupBox_time_vis)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(80, 90, 52, 20))
        self.label_6.setStyleSheet(u"background-color: rgb(255, 255, 255);\n"
"border-color: rgb(85, 0, 255);")
        self.label_6.setFrameShape(QFrame.Shape.Box)
        self.label_6.setFrameShadow(QFrame.Shadow.Raised)
        self.label_6.setLineWidth(1)
        self.label_6.setTextFormat(Qt.TextFormat.PlainText)
        self.label_6.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)
        self.label_6.setIndent(-1)
        self.label_6.setTextInteractionFlags(Qt.TextInteractionFlag.NoTextInteraction)
        self.label_5 = QLabel(self.groupBox_time_vis)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(80, 30, 71, 61))
        self.label_5.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)
        self.label_5.setWordWrap(True)
        self.label_7 = QLabel(self.groupBox_time_vis)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(140, 90, 21, 16))
        self.label_7.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_7.setWordWrap(True)
        self.verticalSlider_TimeWindow = QSlider(self.groupBox_time_vis)
        self.verticalSlider_TimeWindow.setObjectName(u"verticalSlider_TimeWindow")
        self.verticalSlider_TimeWindow.setGeometry(QRect(40, 20, 20, 121))
        self.verticalSlider_TimeWindow.setMinimum(2)
        self.verticalSlider_TimeWindow.setMaximum(60)
        self.verticalSlider_TimeWindow.setSingleStep(8)
        self.verticalSlider_TimeWindow.setPageStep(10)
        self.verticalSlider_TimeWindow.setValue(10)
        self.verticalSlider_TimeWindow.setSliderPosition(10)
        self.verticalSlider_TimeWindow.setTracking(True)
        self.verticalSlider_TimeWindow.setOrientation(Qt.Orientation.Vertical)
        self.verticalSlider_TimeWindow.setInvertedAppearance(False)
        self.verticalSlider_TimeWindow.setInvertedControls(False)
        self.verticalSlider_TimeWindow.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.verticalSlider_TimeWindow.setTickInterval(0)
        self.label_9 = QLabel(self.groupBox_time_vis)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setGeometry(QRect(10, 120, 23, 16))
        self.label_9.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_9.setWordWrap(True)
        self.label_10 = QLabel(self.groupBox_time_vis)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setGeometry(QRect(10, 20, 25, 16))
        self.label_10.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_10.setWordWrap(True)

        self.verticalLayout_3.addWidget(self.groupBox_time_vis)

        self.groupBox_manual_input = QGroupBox(self.tab_experiment)
        self.groupBox_manual_input.setObjectName(u"groupBox_manual_input")
        self.groupBox_manual_input.setMaximumSize(QSize(188, 16777215))
        self.lineEdit_manual_input = QLineEdit(self.groupBox_manual_input)
        self.lineEdit_manual_input.setObjectName(u"lineEdit_manual_input")
        self.lineEdit_manual_input.setGeometry(QRect(110, 60, 41, 21))
        self.lineEdit_manual_input.setFrame(True)
        self.label_manual_input = QLabel(self.groupBox_manual_input)
        self.label_manual_input.setObjectName(u"label_manual_input")
        self.label_manual_input.setGeometry(QRect(10, 60, 91, 21))
        self.label_manual_input.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)
        self.label_manual_input.setWordWrap(True)
        self.horizontalSlider_manual_input = QSlider(self.groupBox_manual_input)
        self.horizontalSlider_manual_input.setObjectName(u"horizontalSlider_manual_input")
        self.horizontalSlider_manual_input.setGeometry(QRect(30, 30, 131, 18))
        self.horizontalSlider_manual_input.setMinimum(-255)
        self.horizontalSlider_manual_input.setMaximum(255)
        self.horizontalSlider_manual_input.setOrientation(Qt.Orientation.Horizontal)
        self.horizontalSlider_manual_input.setInvertedAppearance(False)
        self.horizontalSlider_manual_input.setTickPosition(QSlider.TickPosition.TicksAbove)
        self.horizontalSlider_manual_input.setTickInterval(32)
        self.pushButton_manual_input = QPushButton(self.groupBox_manual_input)
        self.pushButton_manual_input.setObjectName(u"pushButton_manual_input")
        self.pushButton_manual_input.setGeometry(QRect(70, 100, 41, 24))

        self.verticalLayout_3.addWidget(self.groupBox_manual_input)

        self.verticalLayout_3.setStretch(0, 3)
        self.verticalLayout_3.setStretch(1, 2)

        self.horizontalLayout_2.addLayout(self.verticalLayout_3)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.widget_plot_1 = PlotWidget(self.tab_experiment)
        self.widget_plot_1.setObjectName(u"widget_plot_1")
        self.widget_plot_1.setEnabled(True)
        self.widget_plot_1.setStyleSheet(u"border-color: rgb(0, 0, 0);")

        self.verticalLayout_2.addWidget(self.widget_plot_1)

        self.widget_plot_2 = PlotWidget(self.tab_experiment)
        self.widget_plot_2.setObjectName(u"widget_plot_2")
        self.widget_plot_2.setEnabled(True)
        self.widget_plot_2.setStyleSheet(u"border-color: rgb(0, 0, 0);")

        self.verticalLayout_2.addWidget(self.widget_plot_2)


        self.horizontalLayout_2.addLayout(self.verticalLayout_2)

        self.horizontalLayout_2.setStretch(0, 5)
        self.horizontalLayout_2.setStretch(1, 11)

        self.verticalLayout_4.addLayout(self.horizontalLayout_2)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.groupBox_SaveData = QGroupBox(self.tab_experiment)
        self.groupBox_SaveData.setObjectName(u"groupBox_SaveData")
        sizePolicy.setHeightForWidth(self.groupBox_SaveData.sizePolicy().hasHeightForWidth())
        self.groupBox_SaveData.setSizePolicy(sizePolicy)
        self.groupBox_SaveData.setMinimumSize(QSize(361, 70))
        self.groupBox_SaveData.setMaximumSize(QSize(16777215, 70))
        self.horizontalLayout_3 = QHBoxLayout(self.groupBox_SaveData)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.commandLinkButton_save_data = QCommandLinkButton(self.groupBox_SaveData)
        self.commandLinkButton_save_data.setObjectName(u"commandLinkButton_save_data")
        self.commandLinkButton_save_data.setMinimumSize(QSize(125, 40))
        self.commandLinkButton_save_data.setStyleSheet(u"background-color: rgb(254, 255, 196);\n"
"disabled {background-color: rgb(50,50,50)};")
        icon = QIcon(QIcon.fromTheme(QIcon.ThemeIcon.DocumentSave))
        self.commandLinkButton_save_data.setIcon(icon)

        self.horizontalLayout_3.addWidget(self.commandLinkButton_save_data)

        self.label = QLabel(self.groupBox_SaveData)
        self.label.setObjectName(u"label")
        self.label.setScaledContents(False)

        self.horizontalLayout_3.addWidget(self.label)

        self.lcdNumber_dados_coletados = QLCDNumber(self.groupBox_SaveData)
        self.lcdNumber_dados_coletados.setObjectName(u"lcdNumber_dados_coletados")
        self.lcdNumber_dados_coletados.setEnabled(True)
        self.lcdNumber_dados_coletados.setFrameShape(QFrame.Shape.Box)
        self.lcdNumber_dados_coletados.setFrameShadow(QFrame.Shadow.Plain)
        self.lcdNumber_dados_coletados.setLineWidth(1)
        self.lcdNumber_dados_coletados.setMidLineWidth(0)
        self.lcdNumber_dados_coletados.setSmallDecimalPoint(False)
        self.lcdNumber_dados_coletados.setDigitCount(7)
        self.lcdNumber_dados_coletados.setSegmentStyle(QLCDNumber.SegmentStyle.Flat)
        self.lcdNumber_dados_coletados.setProperty(u"value", 0.000000000000000)
        self.lcdNumber_dados_coletados.setProperty(u"intValue", 0)

        self.horizontalLayout_3.addWidget(self.lcdNumber_dados_coletados)


        self.horizontalLayout.addWidget(self.groupBox_SaveData)

        self.pushButton_start = QPushButton(self.tab_experiment)
        self.pushButton_start.setObjectName(u"pushButton_start")
        self.pushButton_start.setMinimumSize(QSize(0, 24))
        self.pushButton_start.setAutoFillBackground(False)
        self.pushButton_start.setStyleSheet(u"background-color: rgb(197, 255, 193);\n"
"font: 600 12pt \"Segoe UI\";")

        self.horizontalLayout.addWidget(self.pushButton_start)

        self.pushButton_stop = QPushButton(self.tab_experiment)
        self.pushButton_stop.setObjectName(u"pushButton_stop")
        self.pushButton_stop.setEnabled(False)
        self.pushButton_stop.setStyleSheet(u"background-color: rgb(255, 203, 194);\n"
"font: 600 12pt \"Segoe UI\";")

        self.horizontalLayout.addWidget(self.pushButton_stop)

        self.horizontalLayout.setStretch(0, 3)
        self.horizontalLayout.setStretch(1, 1)
        self.horizontalLayout.setStretch(2, 1)

        self.verticalLayout_4.addLayout(self.horizontalLayout)

        self.verticalLayout_4.setStretch(0, 5)
        self.verticalLayout_4.setStretch(1, 1)
        self.tabWidge_main.addTab(self.tab_experiment, "")

        self.gridLayout.addWidget(self.tabWidge_main, 0, 0, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        self.tabWidge_main.setCurrentIndex(0)
        self.comboBox.setCurrentIndex(5)
        self.tabWidget.setCurrentIndex(0)
        self.comboBox_ctrl_code.setCurrentIndex(-1)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.groupBox_4.setTitle(QCoreApplication.translate("MainWindow", u"Ajustes de Tempo", None))
        self.lineEdit_duration.setText(QCoreApplication.translate("MainWindow", u"auto", None))
        self.label_17.setText(QCoreApplication.translate("MainWindow", u"Dura\u00e7\u00e3o do Experimento [seg.]:", None))
        self.label_18.setText(QCoreApplication.translate("MainWindow", u"Intervalo de Amostragem:", None))
        self.spinBox_sample_time.setSuffix(QCoreApplication.translate("MainWindow", u" ms", None))
        self.checkBox_confirm_ts.setText(QCoreApplication.translate("MainWindow", u"Confirmar\n"
"(envia para a Plataforma).", None))
        self.groupBox_controlled_output.setTitle(QCoreApplication.translate("MainWindow", u"Vari\u00e1vel Controlada", None))
        self.radioButton_position.setText(QCoreApplication.translate("MainWindow", u"Posi\u00e7\u00e3o Angular [graus]", None))
        self.radioButton_speed.setText(QCoreApplication.translate("MainWindow", u"Velocidade Angular [graus/s]", None))
        self.groupBox_reference_input.setTitle(QCoreApplication.translate("MainWindow", u"Refer\u00eancia ou Entrada em M.A.", None))
        self.radioButton_manual.setText(QCoreApplication.translate("MainWindow", u"Manual", None))
        self.radioButton_prg.setText(QCoreApplication.translate("MainWindow", u"Programada", None))
#if QT_CONFIG(tooltip)
        self.groupBox_input_prg.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p>Entre aqui os valores de comando PWM e os instantes de tempo correspondentes.</p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.groupBox_input_prg.setTitle(QCoreApplication.translate("MainWindow", u"Programa\u00e7\u00e3o da Entrada", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"     Tempo:     Valor:", None))

        __sortingEnabled = self.tableWidget_prog_input.isSortingEnabled()
        self.tableWidget_prog_input.setSortingEnabled(False)
        ___qtablewidgetitem = self.tableWidget_prog_input.item(0, 0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem1 = self.tableWidget_prog_input.item(0, 1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem2 = self.tableWidget_prog_input.item(1, 0)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("MainWindow", u"30", None));
        ___qtablewidgetitem3 = self.tableWidget_prog_input.item(1, 1)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("MainWindow", u"50", None));
        ___qtablewidgetitem4 = self.tableWidget_prog_input.item(2, 0)
        ___qtablewidgetitem4.setText(QCoreApplication.translate("MainWindow", u"60", None));
        ___qtablewidgetitem5 = self.tableWidget_prog_input.item(2, 1)
        ___qtablewidgetitem5.setText(QCoreApplication.translate("MainWindow", u"100", None));
        ___qtablewidgetitem6 = self.tableWidget_prog_input.item(3, 0)
        ___qtablewidgetitem6.setText(QCoreApplication.translate("MainWindow", u"90", None));
        ___qtablewidgetitem7 = self.tableWidget_prog_input.item(3, 1)
        ___qtablewidgetitem7.setText(QCoreApplication.translate("MainWindow", u"150", None));
        ___qtablewidgetitem8 = self.tableWidget_prog_input.item(4, 0)
        ___qtablewidgetitem8.setText(QCoreApplication.translate("MainWindow", u"120", None));
        ___qtablewidgetitem9 = self.tableWidget_prog_input.item(4, 1)
        ___qtablewidgetitem9.setText(QCoreApplication.translate("MainWindow", u"0", None));
        ___qtablewidgetitem10 = self.tableWidget_prog_input.item(5, 0)
        ___qtablewidgetitem10.setText(QCoreApplication.translate("MainWindow", u"150", None));
        ___qtablewidgetitem11 = self.tableWidget_prog_input.item(5, 1)
        ___qtablewidgetitem11.setText(QCoreApplication.translate("MainWindow", u"-50", None));
        ___qtablewidgetitem12 = self.tableWidget_prog_input.item(6, 0)
        ___qtablewidgetitem12.setText(QCoreApplication.translate("MainWindow", u"180", None));
        ___qtablewidgetitem13 = self.tableWidget_prog_input.item(6, 1)
        ___qtablewidgetitem13.setText(QCoreApplication.translate("MainWindow", u"-100", None));
        self.tableWidget_prog_input.setSortingEnabled(__sortingEnabled)

        self.tabWidge_main.setTabText(self.tabWidge_main.indexOf(self.tab_general_configuration), QCoreApplication.translate("MainWindow", u"Vis\u00e3o Geral do Ensaio", None))
#if QT_CONFIG(tooltip)
        self.groupBox_3.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p>Configure e teste a comunica\u00e7\u00e3o com o sistema aqui.</p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.groupBox_3.setTitle(QCoreApplication.translate("MainWindow", u"Comunica\u00e7\u00e3o", None))
        self.pushButton_test_comm.setText(QCoreApplication.translate("MainWindow", u"Testar Comunica\u00e7\u00e3o\n"
"com o Sistema", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"Porta Serial:", None))
        self.label_14.setText(QCoreApplication.translate("MainWindow", u"Taxa de Transm.", None))
        self.comboBox.setItemText(0, QCoreApplication.translate("MainWindow", u"9600 bps", None))
        self.comboBox.setItemText(1, QCoreApplication.translate("MainWindow", u"14400 bps", None))
        self.comboBox.setItemText(2, QCoreApplication.translate("MainWindow", u"19200 bps", None))
        self.comboBox.setItemText(3, QCoreApplication.translate("MainWindow", u"38400 bps", None))
        self.comboBox.setItemText(4, QCoreApplication.translate("MainWindow", u"57600 bps", None))
        self.comboBox.setItemText(5, QCoreApplication.translate("MainWindow", u"115200 bps", None))

        self.comboBox.setCurrentText(QCoreApplication.translate("MainWindow", u"115200 bps", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" text-decoration: underline;\">Obs</span>.: (8) bits de dados, Sem paridade (N), (1) stop bit.</p></body></html>", None))
        self.checkBox_auto_serial_port.setText(QCoreApplication.translate("MainWindow", u"Autom\u00e1tico", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"\u00c1rea de Mensagens: ", None))
        self.tabWidge_main.setTabText(self.tabWidge_main.indexOf(self.tab_comm_serial), QCoreApplication.translate("MainWindow", u"Comunica\u00e7\u00e3o Serial", None))
        self.groupBox_controller_choice.setTitle(QCoreApplication.translate("MainWindow", u"Escolha do Controlador", None))
        self.radioButton_open_loop.setText(QCoreApplication.translate("MainWindow", u"Malha Aberta", None))
        self.radioButton_PID_s.setText(QCoreApplication.translate("MainWindow", u"PID em \"s\"", None))
        self.radioButton_FT_s.setText(QCoreApplication.translate("MainWindow", u"FT em \"s\"", None))
        self.radioButton_PID_discrete.setText(QCoreApplication.translate("MainWindow", u"PID discreto", None))
        self.radioButton_FT_discrete.setText(QCoreApplication.translate("MainWindow", u"FT discreta", None))
        self.radioButton_generic.setText(QCoreApplication.translate("MainWindow", u"Gen\u00e9rico", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"Informa\u00e7\u00e3o sobre o c\u00f3digo do controlador.", None))
#if QT_CONFIG(tooltip)
        self.textEdit_control_info.setToolTip(QCoreApplication.translate("MainWindow", u"Informa\u00e7\u00e3o sobre o tipo de controlador selecionado ao lado.", None))
#endif // QT_CONFIG(tooltip)
        self.groupBox_deadzone_comp.setTitle(QCoreApplication.translate("MainWindow", u"Compensa\u00e7\u00e3o de Zona Morta", None))
        self.label_12.setText("")
        self.label_19.setText(QCoreApplication.translate("MainWindow", u"Cp :", None))
        self.label_20.setText(QCoreApplication.translate("MainWindow", u"Cn :", None))
        self.label_26.setText(QCoreApplication.translate("MainWindow", u"O valor de Cp ser\u00e1 adicionado ao comando PWM, sempre que se computar uma a\u00e7\u00e3o de controle 'u' positiva.\n"
"\n"
" O valor de Cn ser\u00e1 sempre adicionado ao comando PWM, sempre que a a\u00e7\u00e3o de controle 'u' for negativa.", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_general_config), QCoreApplication.translate("MainWindow", u"Configura\u00e7\u00f5es Gerais", None))
        self.label_21.setText("")
        self.groupBox_ct_ti.setTitle(QCoreApplication.translate("MainWindow", u"A\u00e7\u00e3o Integral (pode ser desabilitada)", None))
        self.label_23.setText(QCoreApplication.translate("MainWindow", u"Ti, em [seg.] :", None))
        self.lineEdit_ct_ti.setText(QCoreApplication.translate("MainWindow", u"10", None))
        self.groupBox_ct_td.setTitle(QCoreApplication.translate("MainWindow", u"A\u00e7\u00e3o Derivativa (pode ser desabilitada)", None))
        self.label_25.setText(QCoreApplication.translate("MainWindow", u"Td, em [seg] :", None))
        self.lineEdit_ct_td.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.groupBox_7.setTitle(QCoreApplication.translate("MainWindow", u"A\u00e7\u00e3o Proporcional", None))
        self.label_24.setText(QCoreApplication.translate("MainWindow", u"Kp :", None))
        self.lineEdit_ct_kp.setText(QCoreApplication.translate("MainWindow", u"1", None))
        self.label_22.setText(QCoreApplication.translate("MainWindow", u"Ex.: Kp = 2 indica que, supondo apenas a\u00e7\u00e3o proporcional,\n"
"PWM = 2, quando erro = 1 [deg] (ou 1 [deg/s]).", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_PID_controller), QCoreApplication.translate("MainWindow", u"PID em \"s\"", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_FT_controller), QCoreApplication.translate("MainWindow", u"FT em \"s\"", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"C\u00f3digo C/C++ para c\u00e1lculo da a\u00e7\u00e3o de controle:", None))
        self.plainTextEdit_ctrl_code.setPlainText(QCoreApplication.translate("MainWindow", u"///////////////////////////////////////////////////////////////////\n"
"// Implemente o calculo da acao de controle aqui.\n"
"//\n"
"// Variaveis que voce pode usar (jah estao disponiveis):\n"
"//\n"
"//   t           -> tempo [s]\n"
"//   ctrl_last_t -> ultimo instante de tempo em que o\n"
"//                  codigo do controlador foi executado [s].\n"
"//                  Obs.: (t - ctrl_last_t) deve ser bem proximo\n"
"//                  do intervalo de controle = 10ms.\n"
"//\n"
"//   ym          -> vetor de sinais medidos: ym[0] a ym[3]:\n"
"//                  ym[0] = posicao em [graus].\n"
"//                  ym[1] = velocidade angular [graus/s].\n"
"//\n"
"//\n"
"//   ref         -> vetor de sinais de referencia: ref[0].\n"
"//\n"
"//   u           -> vetor de sinais de entrada da planta u[0].\n"
"//\n"
"//   xc          -> vetor de estados do controlador\n"
"//                  (inicializados com valor zero): xc[0] a xc[5].\n"
"//                  Use para implementar um controlador\n"
"//         "
                        "         dinamico, ao inves de um estatico.\n"
"\n"
"// Ensaio em malha aberta: envia o valor de comando PWM,\n"
"// alterado via interface grafica, diretamente para o motor BLDC NIDEC.\n"
"\n"
"// Aplica o valor diretamente na entrada do sistema\n"
"u[0] = ref[0];\n"
"", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Selecione um Template para o c\u00f3digo do controlador:", None))
        self.comboBox_ctrl_code.setItemText(0, QCoreApplication.translate("MainWindow", u"Malha Aberta", None))
        self.comboBox_ctrl_code.setItemText(1, QCoreApplication.translate("MainWindow", u"P", None))
        self.comboBox_ctrl_code.setItemText(2, QCoreApplication.translate("MainWindow", u"PI", None))

        self.comboBox_ctrl_code.setPlaceholderText(QCoreApplication.translate("MainWindow", u"Lei de Controle", None))
        self.pushButton_ctrl_code.setText(QCoreApplication.translate("MainWindow", u"Carregar Template", None))
        self.pushButton_send_code_ESP32.setText(QCoreApplication.translate("MainWindow", u"Compilar C\u00f3digo e \n"
"Enviar para o ESP32", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_generic_controller), QCoreApplication.translate("MainWindow", u"Gen\u00e9rico", None))
#if QT_CONFIG(tooltip)
        self.tabWidget.setTabToolTip(self.tabWidget.indexOf(self.tab_generic_controller), QCoreApplication.translate("MainWindow", u"Entre o c\u00f3digo de um controlador gen\u00e9rico.", None))
#endif // QT_CONFIG(tooltip)
        self.tabWidge_main.setTabText(self.tabWidge_main.indexOf(self.tab_controller), QCoreApplication.translate("MainWindow", u"Controlador", None))
        self.groupBox_time_vis.setTitle(QCoreApplication.translate("MainWindow", u"Vis. Tempo", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Largura da Janela Temporal", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"6", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Intervalo de Tempo Mostrado:", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"seg.", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"min.", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"m\u00e1x.", None))
        self.groupBox_manual_input.setTitle(QCoreApplication.translate("MainWindow", u"Entrada Manual", None))
#if QT_CONFIG(tooltip)
        self.lineEdit_manual_input.setToolTip(QCoreApplication.translate("MainWindow", u"Entre o valor do comando PWM", None))
#endif // QT_CONFIG(tooltip)
        self.lineEdit_manual_input.setInputMask("")
        self.lineEdit_manual_input.setText("")
        self.label_manual_input.setText(QCoreApplication.translate("MainWindow", u"PWM =", None))
        self.pushButton_manual_input.setText(QCoreApplication.translate("MainWindow", u"Zero", None))
        self.groupBox_SaveData.setTitle(QCoreApplication.translate("MainWindow", u"Dados", None))
        self.commandLinkButton_save_data.setText(QCoreApplication.translate("MainWindow", u"Salvar Dados", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Pontos Coletados:", None))
#if QT_CONFIG(tooltip)
        self.pushButton_start.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p>Iniciar o experimento.</p></body></html>", None))
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(whatsthis)
        self.pushButton_start.setWhatsThis(QCoreApplication.translate("MainWindow", u"<html><head/><body><p>Bot\u00e3o para iniciar o experimento em Malha Aberta.</p></body></html>", None))
#endif // QT_CONFIG(whatsthis)
        self.pushButton_start.setText(QCoreApplication.translate("MainWindow", u"Iniciar", None))
#if QT_CONFIG(tooltip)
        self.pushButton_stop.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p>Parar o experimento.</p></body></html>", None))
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(whatsthis)
        self.pushButton_stop.setWhatsThis(QCoreApplication.translate("MainWindow", u"<html><head/><body><p>Bot\u00e3o para parar a coleta de dados do experimento em Malha Aberta.</p></body></html>", None))
#endif // QT_CONFIG(whatsthis)
        self.pushButton_stop.setText(QCoreApplication.translate("MainWindow", u"Parar", None))
        self.tabWidge_main.setTabText(self.tabWidge_main.indexOf(self.tab_experiment), QCoreApplication.translate("MainWindow", u"Experimento", None))
    # retranslateUi

