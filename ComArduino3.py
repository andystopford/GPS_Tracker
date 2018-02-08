import serial
import time
import sys
import pickle
from PyQt4 import QtGui, uic
# -*- coding: utf-8 -*-

sys.path.append("./Modules")


# from testGUI import Ui_MainWindow


class StartQT4(QtGui.QMainWindow):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.ui = uic.loadUi('Modules/testGUI.ui', self)
        # self.ui = Ui_MainWindow()
        # self.ui.setupUi(self)
        self.item = 0
        self.ui.button1.setText('Open Serial')
        self.ui.button2.setText('Hello')
        self.ui.button3.setText('Delete File')
        self.ui.button4.setText('Read Log')
        self.ui.button5.setText('Close serial')
        # Connect signals to slots
        self.ui.button1.clicked.connect(self.open_serial)
        self.ui.button2.clicked.connect(self.hello)
        self.ui.button3.clicked.connect(self.delete_file)
        self.ui.button4.clicked.connect(self.read_log)
        self.ui.button5.clicked.connect(self.close_serial)
        # self.ui.pasteButton.clicked.connect(self.paste)

        self.ser = serial.Serial()
        self.ser.baudrate = 9600
        # List ports using find_port.py, or auto-connect with
        # usb-ser-mon.py from same source
        self.ser.port = "/dev/ttyUSB0"
        self.ser.timeout = 1

        self.ui.textEdit.setText('050917.log')

    def open_serial(self):
        if self.ser.is_open:
            self.close_serial()
        self.ser.open()
        self.ser.write(00)  # wake it up
        time.sleep(2)
        print('Opening ', self.ser.name)

    def hello(self):
        print('Reading Serial')
        #time.sleep(2)  # let the connection settle
        handshake = '\n' + 'hello' + '\r'  # Key word to instruct Arduino
        msg = handshake.encode('ascii')
        self.ser.write(msg)
        self.read_file_list()

    def delete_file(self):
        print('Deleting')
        name = self.ui.textEdit.toPlainText()
        handshake = '\n' + 'D' + name + '\r'  # Key word to instruct Arduino
        msg = handshake.encode('ascii')
        self.ser.write(msg)
        data = self.ser.readline()
        print(data, ' deleted')

    def read_log(self):
        print('Reading Log')
        name = self.ui.textEdit.toPlainText()
        handshake = '\n' + name + '\r'  # Key word to instruct Arduino
        msg = handshake.encode('ascii')
        self.ser.write(msg)
        # self.read_sd()
        self.read(name)

    def clear_serial(self):
        """Read single line between Line feed and carr. return markers.
        n.b.there's a Python command for this"""
        x = self.ser.read()
        while len(x) == 0:
            x = self.ser.read()
            print('loop')
        print('end')
        self.read()

    def read(self, name):
        path = './Logs/' + name
        data_list = []
        data = self.ser.readline()
        count = 0
        while len(data) == 0:
            if count < 6:
                print('blank received')
                count += 1
                data = self.ser.readline()
            else:
                print('Giving up')
                self.close_serial()
                break
        while data:
            if len(data) > 0:
                text = data.decode('utf-8')
                text = text.strip()
                if len(text) > 0:
                    data_list.append(text)
                    print(text)
                if data == b'\r':
                    # print('end')
                    break
            data = self.ser.readline()
        print('======================')
        if data_list:
            with open(path, 'w') as log_file:
                # pickle.dump(data_list, log_file)
                for line in data_list:
                    log_file.write('%s\n' % line)
            log_file.close()

    def read_file_list(self):
        data_list = []
        data = self.ser.readline()
        count = 0
        while len(data) == 0:
            if count < 6:
                print('blank received')
                count += 1
                data = self.ser.readline()
            else:
                print('Giving up')
                #self.close_serial()
                break
        while data:
            if len(data) > 0:
                text = data.decode('utf-8')
                text = text.strip()
                if len(text) > 0:
                    data_list.append(text)
                    print(text)
                if data == b'\r':
                    # print('end')
                    break
            data = self.ser.readline()
        print('======================')
        print(data_list)


    def read_sd(self):
        """Read lines between < and > markers"""
        start = 60
        end = 62  # i.e. '>'
        data = ''
        x = self.ser.read()
        # save data until the end marker is found:
        print(x)
        while len(x) == 0:
            print('blank received')
            x = self.ser.read()
        while x:
            try:
                if ord(x) == start:
                    # record = True
                    print('start found')
                if ord(x) != start:
                    if ord(x) != end:
                        data = data + x.decode('ascii')
                x = self.ser.read()
                # print("Downloading: ", x)
            except:
                print('Error - close and re-open Serial')
                break
        print(data)

    def close_serial(self):
        self.ser.close()
        print('Serial closed')


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    myapp = StartQT4()
    myapp.show()
    sys.exit(app.exec_())
