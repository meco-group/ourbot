#!/usr/bin/python

import Tkinter as tk
from PIL import Image
from PIL import ImageTk
import threading
import Queue
import sys
import socket
import cv2
import os
import numpy as np

HOST = ''
PORT = 6000

WIDTH = 192
HEIGHT = 108


class OurbotControlCenter:

    def __init__(self, root):
        self.root = root
        self.build_gui()
        self.running = 1
        # set-up threads
        self.video_queue = Queue.Queue()
        self.video_thread = threading.Thread(target=self.video_loop)
        self.video_thread.start()
        # loop in main gui-thread
        self.periodic_call()

    def build_gui(self):
        self.record = False
        self.panel = None
        self.root.wm_title('Ourbot Control Center')
        self.root.wm_protocol('WM_DELETE_WINDOW', self.on_close)
        self.record_btn_text = 'Record'
        self.record_btn = tk.Button(self.root, textvariable=self.record_btn_text, command=self.record_video)
        self.record_btn.pack()
        # console = tk.Button(master, text='Done', command=endCommand)
        # console.pack()
        # Add more GUI stuff here

    def on_close(self):
        self.running = False
        self.root.quit()

    def process_incoming(self):
        # interprete data in queue's
        while self.video_queue.qsize():
            try:
                frame_data = self.video_queue.get(0)
                frame = self.bytes2image(frame_data, WIDTH, HEIGHT)
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = cv2.resize(image, (0, 0), fx=2, fy=2)
                cv2.circle(image, (0, 0), 100, (0, 255, 1), thickness=1, lineType=8)
                if self.record:
                    self.recorder.write(image)
                image = Image.fromarray(image)
                image = ImageTk.PhotoImage(image)
                if self.panel is None:
                    self.panel = tk.Label(image=image)
                    self.panel.image = image
                    self.panel.pack(side="left", padx=10, pady=10)
                else:
                    self.panel.configure(image=image)
                    self.panel.image = image
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except Queue.Empty:
                pass

    def record_video(self):
        if not self.record:
            print 'here!'
            self.record = True
            self.recorder = cv2.VideoWriter('movie.avi', -1, 20.0, (640, 480))
            self.record_btn_text = 'Recording'
        else:
            self.record = False
            self.recorder.release()

    def periodic_call(self): # period in ms
        period = 10
        self.process_incoming()
        if not self.running:
            sys.exit(1)
        self.root.after(period, self.periodic_call)

    def video_loop(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # server_socket.setblocking(0)
        try:
            server_socket.bind((HOST, PORT))
        except socket.error as msg:
            print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
            sys.exit()
        server_socket.listen(10)
        print 'Waiting for client connection...'
        connection, address = server_socket.accept()
        print 'Connected with ' + address[0] + ':' + str(address[1])

        data = ''
        im_size = 3*WIDTH*HEIGHT
        while self.running:
            while len(data) < im_size:
                data += connection.recv(4096)
            frame_data = data[:im_size]
            data = data[im_size:]
            self.video_queue.put(frame_data)
        server_socket.close()

    def bytes2image(self, bytestring, width, height):
        image = np.zeros((height, width, 3), np.uint8)
        cnt = 0
        for i in range(height):
            for j in range(width):
                # image[i, j] = (bytestring[cnt], bytestring[cnt+1], bytestring[cnt+2])
                image[i, j] = (ord(bytestring[cnt]), ord(bytestring[cnt+1]), ord(bytestring[cnt+2]))
                cnt += 3
        return image

    def file_ready(self, filename):
        if not os.path.isfile(filename):
            return False
        try:
            fo = open(filename, 'a', 8)
            if fo:
                return True
        except IOError:
            return False
        finally:
            fo.close()

    def endApplication(self):
        self.running = 0

root = tk.Tk()
occ = OurbotControlCenter(root)
root.mainloop()
