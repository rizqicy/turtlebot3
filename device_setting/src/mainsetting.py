#!/usr/bin/env python
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication
import sys
from setting import Ui_MainWindow
import os

"""
Created on Mon Feb 17 19:59:51 2020

@author: faris
"""
#membuat pewarisan pada Python, dalam kasus ini class Main mewarisi class MainView
#langkah ini dilakukan agar kita dapat melakukan modifikasi pada class MainView tanpa harus mengubah pada class tersebut
#cukup melakukan modifikasi pada class Main
class Main(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(Main,self).__init__()
        #setupUi merupakan fungsi dari class MainView yang berfungsi untuk melakukan setup
        #sesuai dengan apa yang telah didefinisikan sebelumnya
        self.setupUi(self)
        #jalankan fungsi setValue ketika setButtton diclick
        self.setButton1.clicked.connect(self.setValue1)
        self.setButton2.clicked.connect(self.setValue2)
        self.setButton3.clicked.connect(self.setValue3)
        self.setButton4.clicked.connect(self.setValue4)
    #definisikan fungsi setValue
    def setValue1(self):
    #set nilai setLabel dengan nilai setTextEdit
        os.system('roscore')

    def setValue2(self):
        p='roslaunch turtlebot3_bringup covid_robot.launch'
        os.system(p)
    def setValue3(self):
        q='roslaunch turtlebot3_bringup covid_remote.launch'
        os.system(q)
    def setValue4(self):
        r='roslaunch turtlebot3_navigation covid_navigation.launch'
        os.system(r)
def main():
    app = QApplication(sys.argv)  #buat instance baru QtGui
    winForm = Main()                    #inisialisasi variable winForm sebagain class Main
    winForm.show()                      #tampilkan windows form
    sys.exit(app.exec_())               #eksekusi applikasi

if __name__ == '__main__':              #jika file dijalanakan secara langsung, maka jalankan fungsi main()
    main()
