import cv2
import socket
import sys
import pickle
import struct

HOST = ''
PORT = 8089

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    server_socket.bind((HOST, PORT))
except socket.error as msg:
    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
server_socket.listen(10)
print 'Waiting for client connection...'
connection, address = server_socket.accept()
print 'Connected with ' + address[0] + ':' + str(address[1])

video = cv2.VideoCapture(0)
video.set(3, 600)
video.set(4, 400)
while True:
    ret, frame = video.read()
    data = pickle.dumps(frame)
    connection.sendall(struct.pack('L', len(data)) + data)

connection.close()
socket.close()
