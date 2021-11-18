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


import numpy as np
import torch

class NNServer(SocketServer):
    def __init__(
        self,
        host: str = '127.0.0.1',
        port: int = 10051,
        model: torch.nn.Module = None,
        device: torch.device = torch.device('cpu'),
        mean: np.ndarray = None,
        std: np.ndarray = None,
        input_dim: int = 21,
        getImage: callable() = None,
    ):
        super().__init__(host, port)

        self.model = model
        self.device = device
        self.mean = mean if mean is not None else np.zeros(input_dim)
        self.std = std if std is not None else np.ones(input_dim)
        self.input_dim = input_dim

        self.h = torch.zeros((model.LSTM_layer_num, 1, model.LSTM_dim)).to(device)
        self.c = torch.zeros((model.LSTM_layer_num, 1, model.LSTM_dim)).to(device)

    def standby(self):
        return super().standby(self.NN_callback)

    def NN_callback(self, msg:str) -> str:
        # return f'receive "{msg}"'
        data = np.fromstring(msg, dtype=np.float32 , sep=' ')
        # state_dim = 21
        state = data[:self.input_dim]
        # state = np.tile(data[:state_dim], 2)
        print('state shape:', state.shape)
        # image = None
        # while image == None:
        # image = load_image('../repro/video_rgb0/rgb{:.3f}.jpg'.format(data[state_dim]))
        image = self.getImage()
        print('image shape:', image.shape)

        # prediction
        state = (state - self.mean) / self.std

        state = torch.from_numpy(state.astype(np.float32)).to(self.device)
        image = image.to(self.device)

        state = state.unsqueeze(0).unsqueeze(0)
        image = image.unsqueeze(0).unsqueeze(0)

        print('state shape:', state.shape)
        print('image shape:', image.shape)

        state_hat, image_hat, (self.h, self.c) = self.model(state, image, (self.h, self.c))
        # state_hat, image_hat, _, _, (h, c) = model(state, image, (h, c))

        # print(h, c)

        state_hat = state_hat.cpu().detach().numpy().flatten()
        state_hat = state_hat * self.std + self.mean
        print('state_hat:', state_hat)

        image = image.squeeze().cpu().detach().numpy()
        image = image.transpose(1, 2, 0)
        image_hat = image_hat.squeeze().cpu().detach().numpy()
        image_hat = image_hat.transpose(1, 2, 0)
        image_hat = np.concatenate([image, image_hat], axis=1)
        # image_hat_pil = Image.fromarray((225 * image_hat).astype(np.uint8), mode=None)

        # os.makedirs('../repro/video_pred0/', exist_ok=True)
        # image_hat_pil.save('../repro/video_pred0/pred{:.3f}.jpg'.format(data[state_dim]))

        return state_hat.tostring()


def main():
    server = SocketServer()
    server.standby(callback=callback)


if __name__ == '__main__':
    main()
