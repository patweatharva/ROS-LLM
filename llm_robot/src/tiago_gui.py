#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
import rospkg
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QSpacerItem, QSizePolicy
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt

class GuiPublisherNode:
    def __init__(self):
        rospy.init_node('gui_publisher', anonymous=True)
        self.publisher_ = rospy.Publisher('llm_state', String, queue_size=10)
        self.subscriber_ = rospy.Subscriber('llm_state', String, self.update_status)
        self.init_ui()

    def init_ui(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('llm_robot')
        icon_path = package_path + '/resources/microphone_icon.png'

        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle('LLM Robot Assistant')
        self.layout = QVBoxLayout()

        self.title = QLabel('<h1>LLM Robot Assistant</h1>')
        self.description = QLabel('<p>This application allows the robot to listen to user commands.</p>')
        self.status_label = QLabel('<p>Status: Not Listening</p>')

        self.layout.addWidget(self.title)
        self.layout.addWidget(self.description)
        self.layout.addWidget(self.status_label)
        self.layout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        
        self.button = QPushButton(' Start Listening')
        self.button.setIcon(QIcon(icon_path))
        self.button.clicked.connect(self.publish_message)
        self.layout.addWidget(self.button, 0, Qt.AlignHCenter)
        
        self.layout.addItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        self.window.setLayout(self.layout)
        self.window.show()
        sys.exit(self.app.exec_())

    def publish_message(self):
        msg = String()
        msg.data = 'listening'
        self.publisher_.publish(msg)
        rospy.loginfo('Published: "%s"' % msg.data)

    def update_status(self, msg):
        if msg.data == 'listening':
            self.status_label.setText('<p>Status: Listening</p>')
        elif msg.data == 'done listening':
            self.status_label.setText('<p>Status: Not Listening</p>')


def main():
    gui_publisher = GuiPublisherNode()
    rospy.spin()

if __name__ == '__main__':
    main()
