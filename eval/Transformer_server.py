import numpy as np
import torch
from torchvision import transforms
# import cv2
from PIL import Image
import os
from collections import deque


import sys
sys.path.append('.')
from model.SpatialAE import SpatialAE
from SocketServer import SocketServer
from model.TransformerImitation import TransformerImitation
from model.CNNImitation import CNNImitation


class ImageServer():
    def __init__(self, image_size: int = 64):
        self.image_size = image_size

        self.cap = cv2.VideoCapture(0)

        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize(image_size),
            transforms.CenterCrop(image_size),
        ])

        self.getImage()

    def getImage(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = self.transform(frame)
            return frame
        else:
            return torch.zeros((3, self.image_size, self.image_size))


class TransformerServer(SocketServer):
    def __init__(
        self,
        path_AE_param: str,
        path_Transformer_param: str,
        path_output_image: str,
        host: str = '127.0.0.1',
        port: int = 10051,
        device: torch.device = torch.device('cpu'),
        input_dim: int = 48,
        getImage: callable = None,
        memory_size: int = 1000,
    ):
        self.device = device
        self.input_dim = input_dim
        self.getImage = getImage
        self.frames = []
        self.memory_size = memory_size

        # load Spatial Auto Encoder
        self.spatialAE = SpatialAE()
        state_dict = self.load_model_param(path_AE_param)
        self.spatialAE.load_state_dict(state_dict)
        self.spatialAE = self.spatialAE.to(device)
        self.image_encoder = self.spatialAE.encoder
        self.image_decoder = self.spatialAE.decoder

        # load Transformer
        self.policy = TransformerImitation(dim=input_dim)
        state_dict = self.load_model_param(path_Transformer_param)
        self.policy.load_state_dict(state_dict)
        self.policy = self.policy.to(device)

        # load CNN
        # self.cnn = CNNImitation(dim=input_dim)
        # state_dict = self.load_model_param(path_Transformer_param)
        # self.cnn.load_state_dict(state_dict)
        # self.cnn = self.cnn.to(device)

        # self.memory = [torch.zeros((1, 1, input_dim)).to(device)] * memory_size
        # self.memory = deque(self.memory, maxlen=memory_size)
        # self.memory = None
        self.memory = []

        self.path_output_image = path_output_image
        os.makedirs(path_output_image, exist_ok=True)

        self.state_hat_old = None

        super().__init__(host, port)

    def standby(self):
        return super().standby(self.NN_callback)

    def NN_callback(self, msg: str) -> str:
        print('msg', msg)
        data = np.fromstring(msg, dtype=np.float32 , sep=' ')
        # state = data[:self.input_dim]
        state = data[:self.input_dim // 2]

        # format
        state = torch.from_numpy(state.astype(np.float32)).to(self.device)
        state = state.repeat(2)
        state[-8:-1] = -state[-8:-1]
        state = state.unsqueeze(0).unsqueeze(0)

        # if self.memory is None:
        #     self.memory = [state] * self.memory_size
        #     self.memory = deque(self.memory, maxlen=self.memory_size)
        # else:
        #     self.memory.append(state)
        # memory = torch.cat(list(self.memory), dim=1)

        self.memory.append(state)
        memory = torch.cat(self.memory, dim=1)

        print('state shape:', state.shape)
        print('memory shape:', memory.shape)

        if self.getImage is not None:
            image = self.getImage()
            image = image.to(self.device)
            image = image.unsqueeze(0)
            print('image shape:', image.shape)

        # image_feature = self.image_encoder(image)
        # state = torch.cat([state, image_feature.unsqueeze(0)], dim=2)

        # prediction
        state_hat = self.policy(memory)[:, -1]

        # image_hat = self.image_decoder(
        #     image_feature, image_size=image.shape[-1])

        state_hat = state_hat.cpu().detach().numpy().flatten()
        state_hat = state_hat[24:48]
        print('state_hat:', state_hat.shape)
        print('state_hat:', state_hat)

        if self.getImage is not None:
            image = image.squeeze().cpu().detach().numpy()
            image = image.transpose(1, 2, 0)
            frame = image
            frame = Image.fromarray((255 * frame).astype(np.uint8), mode=None)
            frame.save(os.path.join(
                self.path_output_image, f'pred{data[-1]:.3f}.jpg'))

        # temporary
        state_hat = np.delete(state_hat, [2, 10, 18])
        print('state_hat:', state_hat.shape)

        # LPF
        if self.state_hat_old is None:
            self.state_hat_old = state_hat
        state_hat = self.state_hat_old * 0.2 + state_hat * 0.8
        self.state_hat_old = state_hat

        # to string
        msg = ','.join(state_hat.astype(str))
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
        frames = [
            Image.fromarray(frame).quantize(colors=256, method=0, dither=1)
            for frame in self.frames
        ]
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

    # imageServer = ImageServer(image_size=128)

    server = TransformerServer(
        device=device,
        path_AE_param=args.model_AE,
        path_Transformer_param=args.model,
        path_output_image=args.path_output_image,
        # getImage=imageServer.getImage,
        memory_size=100,
    )
    server.standby()
    # server.save_gif('result.gif')


def argparse():
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--model_AE', type=str,
        default='./model_param/SpatialAE_param.pt')
    parser.add_argument('--model', type=str,
        default='./model_param/Transformer_param.pt')
    parser.add_argument('--path_output_image', type=str,
        default='./results/images/')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = argparse()
    main(args)
