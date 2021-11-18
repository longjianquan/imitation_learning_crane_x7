import socket

serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = '127.0.0.1'
serverSocket.connect((host, 10051))

msg = serverSocket.recv(1024)
print(msg.decode('utf-8'))

line = ''
while line != 'exit':
    line = input('>>> ')
    serverSocket.send(line.encode('utf-8'))
    responce = serverSocket.recv(4096).decode()
    print(f'responce: {responce}')

serverSocket.close()
print('Dissconnect from server')
