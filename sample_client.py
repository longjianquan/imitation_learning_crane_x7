import socket
import time

serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = '127.0.0.1'
serverSocket.connect((host, 10051))

msg = serverSocket.recv(1024)
print(msg.decode('utf-8'))

line = ''
while line != 'exit':
    line = input('>>> ')
    if line == '':
        line = ' '.join(['0'] * 21)
    print(f'send: {line}')
    serverSocket.send(line.encode('utf-8'))
    responce = serverSocket.recv(4096).decode()
    print(f'responce: {responce}')
    time.sleep(40)

serverSocket.close()
print('Dissconnect from server')
