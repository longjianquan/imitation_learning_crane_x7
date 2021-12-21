import socket

import numpy as np
import torch
from torchvision import transforms
# import cv2
from PIL import Image
import os

import sys
sys.path.append('.')
from model.SpatialAE import SpatialAE
from model.LSTMImitation import LSTMImitation
from SocketServer import SocketServer


# class SocketServer():
#     def __init__(
#         self,
#         host: str = '127.0.0.1',
#         port: int = 10051,
#         backlog: int = 10,
#         bufsize: int = 4096,
#     ):

#         self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         if host == '':
#             host = socket.gethostname()
#         self.socket.bind((host, port))
#         self.socket.listen(backlog)
#         print(f'Server is listening on {host}:{port}')
#         self.bufsize = bufsize

#         # accept connection
#         print('Waiting for connection...')
#         self.clientSocket, address = self.socket.accept()
#         print(f'Connection from {address} has been established!')
#         self.clientSocket.send(f'Connected to {host}:{port}'.encode('utf-8'))

#     def standby(self, callback: callable(str)):
#         try:
#             while True:
#                 # receive message
#                 msg = self.clientSocket.recv(self.bufsize).decode('utf-8')
#                 print(f'receive "{msg}"')

#                 if msg == 'exit':
#                     break

#                 # calculate callback
#                 output = callback(msg).encode('utf-8')

#                 # send message
#                 self.clientSocket.send(output)
#                 print(f'send "{output}"')

#         finally:
#             self.socket.close()
#             self.clientSocket.close()
#             print('Disconnect from client')


# def callback(msg: str) -> str:
#     return f'receive "{msg}"'


# class ImageServer():
#     def __init__(self, image_size: int = 64):
#         self.cap = cv2.VideoCapture(0)

#         self.transform = transforms.Compose([
#             transforms.ToTensor(),
#             transforms.Resize(image_size),
#             transforms.CenterCrop(image_size),
#         ])

#         self.getImage()

#     def getImage(self):
#         ret, frame = self.cap.read()
#         frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         frame = self.transform(frame)
#         return frame


class AE_LSTM_Server(SocketServer):
    def __init__(
        self,
        path_AE_param: str,
        path_LSTM_param: str,
        host: str = '127.0.0.1',
        port: int = 10051,
        device: torch.device = torch.device('cpu'),
        mean: np.ndarray = None,
        std: np.ndarray = None,
        input_dim: int = 48,
        getImage: callable = None,
        output_dir: str = './results/images/',
    ):
        self.device = device
        self.mean = mean if mean is not None else np.zeros(input_dim)
        self.std = std if std is not None else np.ones(input_dim)
        self.input_dim = input_dim
        self.getImage = getImage
        self.frames = []

        # load Spatial Auto Encoder
        self.spatialAE = SpatialAE()
        state_dict = self.load_model_param(path_AE_param)
        self.spatialAE.load_state_dict(state_dict)
        self.spatialAE = self.spatialAE.to(device)
        self.image_encoder = self.spatialAE.encoder
        self.image_decoder = self.spatialAE.decoder

        # load LSTM
        # image_feature_dim = 32
        image_feature_dim = 0
        self.lstm = LSTMImitation(
            input_dim=input_dim+image_feature_dim,
            output_dim=input_dim,
            LSTM_dim=400,
            LSTM_layer_num=6,
        )
        state_dict = self.load_model_param(path_LSTM_param)
        self.lstm.load_state_dict(state_dict)
        self.lstm = self.lstm.to(device)

        self.h = torch.zeros((self.lstm.LSTM_layer_num, 1, self.lstm.LSTM_dim)).to(device)
        self.c = torch.zeros((self.lstm.LSTM_layer_num, 1, self.lstm.LSTM_dim)).to(device)

        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        super().__init__(host, port)

    def standby(self):
        return super().standby(self.NN_callback)

    def NN_callback(self, msg: str) -> str:
        # return f'receive "{msg}"'
        data = np.fromstring(msg, dtype=np.float32 , sep=' ')
        # state_dim = 21
        # state = data[:self.input_dim]
        # state = np.tile(data[:state_dim], 2)

        state = data[:self.input_dim // 2]

        # format
        state = torch.from_numpy(state.astype(np.float32)).to(self.device)
        state = state.repeat(2)
        state[-8:-1] = -state[-8:-1]
        state = state.unsqueeze(0).unsqueeze(0)
        print('state shape:', state.shape)
        
        # image = None
        # while image == None:
        # image = load_image('../repro/video_rgb0/rgb{:.3f}.jpg'.format(data[state_dim]))
        if self.getImage is not None:
            image = self.getImage()
            print('image shape:', image.shape)
            image = image.to(self.device)
            print('image shape:', image.shape)
            image = image.unsqueeze(0)

        # prediction
        # state = (state - self.mean) / self.std
        # state = torch.from_numpy(state.astype(np.float32)).to(self.device)
        # state = state.unsqueeze(0).unsqueeze(0)
        print('state shape:', state.shape)
        
        # image_feature = self.image_encoder(image)
        # state = torch.cat([state, image_feature.unsqueeze(0)], dim=2)
        
        state_hat, (self.h, self.c) = self.lstm(state, (self.h, self.c))
        
        # image_hat = self.image_decoder(
        #     image_feature, image_size=image.shape[-1])

        state_hat = state_hat.cpu().detach().numpy().flatten()
        # state_hat = state_hat * self.std + self.mean
        print('state_hat:', state_hat)

        if self.getImage is not None:
            image = image.squeeze().cpu().detach().numpy()
            image = image.transpose(1, 2, 0)
            # image_hat = image_hat.squeeze().cpu().detach().numpy()
            # image_hat = image_hat.transpose(1, 2, 0)
            # frame = np.concatenate([image, image_hat], axis=1)
            # self.writer.writeFrame(frame)
            # self.frames.append((255 * frame).astype(np.uint8))
            frame = image
            frame = Image.fromarray((255 * frame).astype(np.uint8), mode=None)
            frame.save(self.output_dir + f'/pred{data[-1]:.3f}.jpg')

        state_hat = state_hat[24:]

        # temporary
        state_hat = np.delete(state_hat, [2, 10, 18])
        print('state_hat:', state_hat.shape)

        # to string
        msg = ''
        for y_element in state_hat[:self.input_dim]:
            msg += f'{y_element.item()},'
        return msg

    def load_model_param(self, filepath):
        state_dict = torch.load(filepath, map_location=self.device)
        from collections import OrderedDict
        new_state_dict = OrderedDict()
        for k, v in state_dict.items():
            if 'module' in k:
                k = k.replace('module.', '')
            new_state_dict[k] = v
        return new_state_dict

    def save_gif(self, path: str, duration: int = 40):
        frames = [Image.fromarray(frame).quantize(colors=256, method=0, dither=1)
            for frame in self.frames]
        frames[0].save(
            path,
            save_all=True,
            append_images=frames[1:],
            duration=duration,
            loop=0
        )


def main(args):
    # pytorch device setting
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print('device:', device)

    # server = SocketServer()
    # server.standby(callback=callback)

    # imageServer = ImageServer(image_size=128)

    server = AE_LSTM_Server(
        device=device,
        path_AE_param=args.path_AE_param,
        path_LSTM_param=args.model,
        # getImage=imageServer.getImage,
    )
    server.standby()
    # server.save_gif('result.gif')

def argparse():
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--path_AE_param', type=str,
        default='./model_param/SpatialAE_param.pt')
    parser.add_argument('--model', type=str,
        default='./model_param/LSTM_param.pt')
    parser.add_argument('--path_output_image', type=str,
        default='./results/images/')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = argparse()
    main(args)
