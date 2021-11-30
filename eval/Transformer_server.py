import numpy as np
import torch
from torchvision import transforms
import cv2
from PIL import Image
import os
from collections import deque


import sys
sys.path.append('.')
from model.SpatialAE import SpatialAE
from SocketServer import SocketServer
from model.TransformerImitation import TransformerImitation


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
        input_dim: int = 24,
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
        self.transformer = TransformerImitation(dim=input_dim)
        state_dict = self.load_model_param(path_Transformer_param)
        self.transformer.load_state_dict(state_dict)
        self.transformer = self.transformer.to(device)

        # self.memory = [torch.zeros((1, 1, input_dim)).to(device)] * memory_size
        # self.memory = deque(self.memory, maxlen=memory_size)
        self.memory = None

        self.path_output_image = path_output_image
        os.makedirs(path_output_image, exist_ok=True)

        super().__init__(host, port)

    def standby(self):
        return super().standby(self.NN_callback)

    def NN_callback(self, msg: str) -> str:
        print('msg', msg)
        data = np.fromstring(msg, dtype=np.float32 , sep=' ')
        state = data[:self.input_dim]
        image = self.getImage()

        # format
        state = torch.from_numpy(state.astype(np.float32)).to(self.device)
        image = image.to(self.device)

        state = state.unsqueeze(0).unsqueeze(0)
        if self.memory is None:
            self.memory = [state] * self.memory_size
            self.memory = deque(self.memory, maxlen=self.memory_size)
        else:
            self.memory.append(state)
        memory = torch.cat(list(self.memory), dim=1)

        print('state shape:', state.shape)
        print('image shape:', image.shape)
        print('memory shape:', memory.shape)

        image = image.unsqueeze(0)
        # image_feature = self.image_encoder(image)
        # state = torch.cat([state, image_feature.unsqueeze(0)], dim=2)

        # prediction
        state_hat = self.transformer(memory)[:, -1]

        # image_hat = self.image_decoder(
        #     image_feature, image_size=image.shape[-1])

        print('state_hat:', state_hat.shape)
        state_hat = state_hat.cpu().detach().numpy().flatten()
        print('state_hat:', state_hat)

        image = image.squeeze().cpu().detach().numpy()
        image = image.transpose(1, 2, 0)
        frame = image
        frame = Image.fromarray((225 * frame).astype(np.uint8), mode=None)
        frame.save(os.path.join(
            self.path_output_image, f'pred{data[-1]:.3f}.jpg'))

        # temporary
        state_hat = np.delete(state_hat, [2, 10, 18])
        print('state_hat:', state_hat.shape)

        # to string
        msg = ''
        for y_element in state_hat:
            if y_element == state_hat[-1]:
                msg += f'{y_element.item()}'
            else:
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

    imageServer = ImageServer(image_size=128)

    server = TransformerServer(
        device=device,
        path_AE_param=args.path_AE_param,
        path_Transformer_param=args.path_Transformer_param,
        path_output_image=args.path_output_image,
        getImage=imageServer.getImage,
    )
    server.standby()
    # server.save_gif('result.gif')


def argparse():
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('--path_AE_param', type=str,
        default='./model_param/SpatialAE_param.pt')
    parser.add_argument('--path_Transformer_param', type=str,
        default='./model_param/Transformer_param.pt')
    parser.add_argument('--path_output_image', type=str,
        default='./results/images/')
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = argparse()
    main(args)
