import socket


class SocketNode():
    def __init__(self, host: str = '127.0.0.1',
                 port: int = 10051, backlog: int = 10):

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if host == '':
            host = socket.gethostname()
        self.s.bind((host, port))
        self.s.listen(backlog)
        print(f'Server is listening on {host}:{port}')
        self.bufsize = 4096
        self.clientsockets = []

    def standby(self, callback: callable(str)):
        try:
            while True:
                # accept connection
                clientsocket, address = self.s.accept()
                print(f'Connection from {address} has been established!')
                clientsocket.send(bytes('Welcome to the server!', 'utf-8'))
                self.clientsockets.append(clientsocket)

                # receive message
                for clientsocket in self.clientsockets:
                    msg = clientsocket.recv(self.bufsize)
                    output = callback(msg.decode('utf-8'))
                    clientsocket.send(bytes(output, 'utf-8'))
                    # clientsocket.close()

        finally:
            self.s.close()
            for cliantsockets in self.clientsockets:
                cliantsockets.close()


def callback(msg: str) -> str:
    return msg


def main():
    node = SocketNode()
    node.standby(callback=callback)


if __name__ == '__main__':
    main()
