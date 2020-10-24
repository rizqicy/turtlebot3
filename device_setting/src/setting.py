#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'setting.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(10, 50, 481, 341))
        self.tabWidget.setObjectName("tabWidget")
        self.DeviceSettings = QtWidgets.QWidget()
        self.DeviceSettings.setObjectName("DeviceSettings")
        self.setButton1 = QtWidgets.QPushButton(self.DeviceSettings)
        self.setButton1.setGeometry(QtCore.QRect(140, 10, 181, 51))
        self.setButton1.setObjectName("setButton1")
        self.setButton2 = QtWidgets.QPushButton(self.DeviceSettings)
        self.setButton2.setGeometry(QtCore.QRect(140, 80, 181, 51))
        self.setButton2.setObjectName("setButton2")
        self.setButton3 = QtWidgets.QPushButton(self.DeviceSettings)
        self.setButton3.setGeometry(QtCore.QRect(140, 160, 181, 51))
        self.setButton3.setObjectName("setButton3")
        self.setButton4 = QtWidgets.QPushButton(self.DeviceSettings)
        self.setButton4.setGeometry(QtCore.QRect(140, 230, 181, 51))
        self.setButton4.setObjectName("setButton4")
        self.tabWidget.addTab(self.DeviceSettings, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.tabWidget.addTab(self.tab_2, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.setButton1.setText(_translate("MainWindow", "ROS Master"))
        self.setButton2.setText(_translate("MainWindow", "Robot Init"))
        self.setButton3.setText(_translate("MainWindow", "Teleoperation Mode"))
        self.setButton4.setText(_translate("MainWindow", "Navigation Mode"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.DeviceSettings), _translate("MainWindow", "Device Settings"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Tab 2"))

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())