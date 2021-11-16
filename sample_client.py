import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = '127.0.0.1'
s.connect((host, 10051))
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

msg = s.recv(1024)
print(msg.decode('utf-8'))

s.send(bytes('Welcome to the server!', 'utf-8'))
