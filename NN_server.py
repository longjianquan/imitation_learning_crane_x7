import socket


class SocketServer():
    def __init__(
        self,
        host: str = '127.0.0.1',
        port: int = 10051,
        backlog: int = 10,
        bufsize: int = 4096,
    ):

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if host == '':
            host = socket.gethostname()
        self.socket.bind((host, port))
        self.socket.listen(backlog)
        print(f'Server is listening on {host}:{port}')
        self.bufsize = bufsize

        # accept connection
        print('Waiting for connection...')
        self.clientSocket, address = self.socket.accept()
        print(f'Connection from {address} has been established!')
        self.clientSocket.send(f'Connected to {host}:{port}'.encode('utf-8'))

    def standby(self, callback: callable(str)):
        try:
            while True:
                # receive message
                msg = self.clientSocket.recv(self.bufsize).decode('utf-8')
                print(f'receive "{msg}"')

                if msg == 'exit':
                    break

                output = callback(msg)
                self.clientSocket.send(output.encode('utf-8'))

        finally:
            self.socket.close()
            self.clientSocket.close()
            print('Disconnect from client')


def callback(msg: str) -> str:
    return f'receive "{msg}"'


def main():
    server = SocketServer()
    server.standby(callback=callback)


if __name__ == '__main__':
    main()
