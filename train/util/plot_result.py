from re import T
from typing import List
import numpy as np
from PIL import Image
import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
import seaborn as sns
sns.set()
from sklearn.decomposition import PCA
import torch


def torch2numpy(tensor):
    return tensor.cpu().detach().numpy().copy()


def rad2deg(rad):
    return rad * 180 / np.pi


def plot_state(
    state_ans: np.ndarray,
    state_hat: np.ndarray,
    DoF: int = 8,
    color1: str = 'tab:grey',
    color2: str = 'tab:blue',
    label1: str = 'teacher',
    label2: str = 'predict',
):
    fig, ax = plt.subplots(
        DoF, 3,
        figsize=(20, 20),
        sharex=True,
        sharey='col',
    )

    state_ans = state_ans.transpose()
    state_hat = state_hat.transpose()

    theta_ans = rad2deg(state_ans[:DoF])
    omega_ans = rad2deg(state_ans[DoF:2*DoF])
    tau_ans = state_ans[2*DoF:3*DoF]
    theta_hat = rad2deg(state_hat[:DoF])
    omega_hat = rad2deg(state_hat[DoF:2*DoF])
    tau_hat = state_hat[2*DoF:3*DoF]

    for i in range(DoF):
        ax[i, 0].plot(theta_ans[i], color=color1, label=label1, alpha=0.8)
        ax[i, 0].plot(theta_hat[i], color=color2, label=label2, alpha=0.8)
        ax[i, 0].set_ylabel(r'$\theta_' + str(i) + '$ [deg]')
        ax[i, 0].set_ylim([-10, 370])
        ax[i, 0].set_yticks(range(0, 370, 90))

        ax[i, 1].plot(omega_ans[i], color=color1, label=label1, alpha=0.8)
        ax[i, 1].plot(omega_hat[i], color=color2, label=label2, alpha=0.8)
        ax[i, 1].set_ylabel(r'$\.{\theta}_' + str(i) + '$ [deg/s]')

        ax[i, 2].plot(tau_ans[i], color=color1, label=label1, alpha=0.8)
        ax[i, 2].plot(tau_hat[i], color=color2, label=label2, alpha=0.8)
        ax[i, 2].set_ylabel(r'$\tau_' + str(i) + '$ [Nm]')

    for i in range(3):
        ax[DoF-1, i].set_xlabel('time step')
    ax[DoF-1, 0].legend(loc='lower left')
    fig.align_labels()
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    return fig


def formatImages(image):
    image = torch2numpy(image).astype(np.float32)
    image = image.transpose(0, 2, 3, 1)
    return image


def plot_generated_image(fig, images_ans, images_hat,
                         keypoints=[], keypoints_hat=[]):
    col = 4
    interval = -(-len(images_ans) // 16)
    idx = np.arange(len(images_ans))[::interval]
    if len(idx) < 16:
        idx = np.append(idx, len(images_ans) - 1)
    images_ans = images_ans[idx]
    images_hat = images_hat[idx]

    steps, im_size, _, channel = images_ans.shape
    if len(keypoints) > 0:
        keypoints = keypoints[idx]
        keypoints = im_size * (keypoints / 2 + 0.5)
        steps, keypoints_dim = keypoints.shape
        keypoints_num = keypoints_dim//2
        keypoints_x = keypoints[:, :keypoints_num]
        keypoints_y = keypoints[:, keypoints_num:]
    if len(keypoints_hat) > 0:
        keypoints_hat = keypoints_hat[idx]
        keypoints_hat = im_size * (keypoints_hat / 2 + 0.5)
        steps, keypoints_dim = keypoints.shape
        keypoints_num = keypoints_dim//2
        keypoints_hat_x = keypoints_hat[:, :keypoints_num]
        keypoints_hat_y = keypoints_hat[:, keypoints_num:]

    row = -(-len(images_ans) // col)
    for i, (image_ans, image_hat) in enumerate(zip(images_ans, images_hat)):
        ax = fig.add_subplot(row, 2 * col, 2 * i + 1)
        ax.imshow(image_ans)
        ax.axis('off')
        ax.set_title('$i_{' + str(idx[i]) + '}$')

        if len(keypoints) > 0:
            ax.scatter(keypoints_x[i], keypoints_y[i],
                       color='tab:blue', alpha=0.5)
        if len(keypoints_hat) > 0:
            ax.scatter(keypoints_hat_x[i], keypoints_hat_y[i],
                       color='tab:red', alpha=0.5)

        ax = fig.add_subplot(row, 2 * col, 2 * i + 2)
        ax.imshow(image_hat)
        ax.axis('off')
        ax.set_title(r'$\hat{i}_{' + str(idx[i]) + '}$')


def save_gif(frames: List[np.array], path: str, duration: int = 20):
    frames = [Image.fromarray(frame).quantize(colors=256, method=0, dither=1)
        for frame in frames]
    frames[0].save(
        path,
        save_all=True,
        append_images=frames[1:],
        duration=duration,
        loop=0
    )


# def numpy2image(array):
#     return Image.fromarray(np.uint8(array * 256))


def make_generated_image_gif(images_ans, images_hat, path):
    frames = []
    for image_ans, image_hat in zip(images_ans, images_hat):
        image_cat = np.concatenate([image_ans, image_hat], axis=1)
        # frames.append(numpy2image(image_cat))
        image_int = np.uint8(image_cat * 255)
        frames.append(image_int)
    save_gif(frames, path, duration=40)


def plot_latent_space(
    fig: mpl.figure.Figure,
    zs: np.ndarray,
    labels: np.ndarray,
    epoch: int = 0,
):
    # zs = torch2numpy(zs)
    # labels = torch2numpy(labels)

    if zs.shape[1] > 2:
        pca = PCA()
        pca.fit(zs)
        zs = pca.transform(zs)
    # zs = TSNE(n_components=2, random_state=0).fit_transform(zs)

    ax = fig.add_subplot(111)
    im = ax.scatter(zs[:, 0], zs[:, 1], c=labels, cmap='jet', marker='.')
    lim = np.max(np.abs(zs)) * 1.1
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    from mpl_toolkits.axes_grid1 import make_axes_locatable
    divider = make_axes_locatable(ax)
    cax = divider.append_axes('right', size='5%', pad=0.1)
    fig.colorbar(im, ax=ax, orientation='vertical', cax=cax)
    ax.set_title('{} epoch'.format(epoch))


def plot_loss(ax, train_loss, valid_loss):
    ax.plot(train_loss, label='train', alpha=0.8)
    ax.plot(valid_loss, label='valid', alpha=0.8)
    train_max = np.mean(train_loss) + 2 * np.std(train_loss)
    valid_max = np.mean(valid_loss) + 2 * np.std(valid_loss)
    y_max = max(train_max, valid_max)
    y_min = min(min(train_loss), min(valid_loss))
    ax.set_ylim(0.9 * y_min, 1.1 * y_max)
    ax.set_yscale('log')


def plot_reconstructed_image(fig, images_ans, images_hat, col=4, epoch=0):
    image_num = int(col**2)
    images_ans = images_ans[:image_num]
    images_hat = images_hat[:image_num]

    cmap = None
    channel = images_ans.shape[3]
    if channel == 1:
        # cmap = 'gray'
        cmap = 'binary'
        images_ans = np.squeeze(images_ans)
        images_hat = np.squeeze(images_hat)

    row = -(-len(images_ans) // col)
    for i, (image_ans, image_hat) in enumerate(zip(images_ans, images_hat)):
        ax = fig.add_subplot(row, 2 * col, 2 * i + 1)
        ax.imshow(image_ans, cmap=cmap)
        ax.axis('off')
        # ax.set_title('$i_{' + str(i) + '}$')

        ax = fig.add_subplot(row, 2 * col, 2 * i + 2)
        ax.imshow(image_hat, cmap=cmap)
        ax.axis('off')
        # ax.set_title(r'$\hat{i}_{' + str(i) + '}$')

    fig.suptitle('{} epoch'.format(epoch))
    fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)


def plot_2D_Manifold(fig, model, device, z_sumple, col=10, epoch=0,
                     label=None, label_transform=lambda x: x, image_size=64):
    row = col

    x = np.tile(np.linspace(-2, 2, col), row)
    y = np.repeat(np.linspace(2, -2, row), col)
    z = np.stack([x, y]).transpose()
    zeros = np.zeros(shape=(z.shape[0], z_sumple.shape[1] - z.shape[1]))
    z = np.concatenate([z, zeros], axis=1)

    if z_sumple.shape[1] > 2:
        z_sumple = torch2numpy(z_sumple)
        pca = PCA()
        pca.fit(z_sumple)
        z = pca.inverse_transform(z)
    z = torch.from_numpy(z.astype(np.float32)).to(device)

    if not label == None:
        label_transformed = label_transform(label)
        label_transformed = label_transformed.repeat(z.shape[0], 1).to(device)
    else:
        label_transformed = None

    images = model.decoder(z, image_size=image_size, label=label_transformed)
    images = formatImages(images)

    cmap = None
    channel = images.shape[3]
    if channel == 1:
        # cmap = 'gray'
        cmap = 'binary'
        images = np.squeeze(images)

    for i, image in enumerate(images):
        ax = fig.add_subplot(row, col, i + 1)
        ax.imshow(image, cmap=cmap)
        ax.axis('off')

    suptitle = '{} epoch'.format(epoch)
    if not label == None:
        suptitle += '  label: {}'.format(label)
    fig.suptitle(suptitle)
    fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)


def plot_latent_traversal(fig, model, device, row, col=10, epoch=0, label=None,
                          label_transform=lambda x: x, image_size=64):
    gradation = np.linspace(-2, 2, col)
    z = np.zeros(shape=(row, col, row))
    for i in range(row):
        z[i, :, i] = gradation
    z = z.reshape(-1, row)
    z = torch.from_numpy(z.astype(np.float32)).to(device)

    if not label == None:
        label_transformed = label_transform(label)
        label_transformed = label_transformed.repeat(z.shape[0], 1).to(device)
    else:
        label_transformed = None

    images = model.decoder(z, image_size=image_size, label=label_transformed)
    images = formatImages(images)

    cmap = None
    channel = images.shape[3]
    if channel == 1:
        # cmap = 'gray'
        cmap = 'binary'
        images = np.squeeze(images)

    for i, image in enumerate(images):
        ax = fig.add_subplot(row, col, i + 1)
        ax.imshow(image, cmap=cmap)
        ax.axis('off')

    suptitle = '{} epoch'.format(epoch)
    if not label == None:
        suptitle += '  label: {}'.format(label)
    fig.suptitle(suptitle)
    fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)


def plot_losses(fig, train_loss, valid_loss,
                train_loss_mse, valid_loss_mse,
                train_loss_ssim, valid_loss_ssim,
                train_loss_kl, valid_loss_kl):
    ax = fig.add_subplot(411)
    plot_loss(ax, train_loss, valid_loss)
    ax.set_ylabel('Total Loss')
    ax.tick_params(bottom=False, labelbottom=False)
    ax.legend()

    ax = fig.add_subplot(412)
    plot_loss(ax, train_loss_mse, valid_loss_mse)
    ax.set_ylabel('Mean Squared Error')
    ax.tick_params(bottom=False, labelbottom=False)

    ax = fig.add_subplot(413)
    plot_loss(ax, train_loss_ssim, valid_loss_ssim)
    ax.set_ylabel('Structural Similarity')
    ax.tick_params(bottom=False, labelbottom=False)

    ax = fig.add_subplot(414)
    plot_loss(ax, train_loss_kl, valid_loss_kl)
    ax.set_ylabel('KL Divergence')
    ax.set_xlabel('epoch')

    fig.align_labels()


def plot_images(
    fig: mpl.figure.Figure,
    images: np.ndarray,
    epoch: int = 0,
    feature_points: list = [],
):
    n_image, height, width, channel = images.shape
    col = np.ceil(np.sqrt(n_image)).astype(np.int)
    row = col

    if len(feature_points) > 0:
        feature_points_dim = len(feature_points)
        feature_points_num = feature_points_dim // 2
        feature_points_x = feature_points[:feature_points_num]
        feature_points_y = feature_points[feature_points_num:]
        feature_points_x = width * (feature_points_x + 1) / 2
        feature_points_y = height * (feature_points_y + 1) / 2

    cmap = None
    if channel == 1:
        cmap = 'gray'
        # cmap = 'binary'
        images = np.squeeze(images)

    for i, image in enumerate(images):
        ax = fig.add_subplot(row, col, i + 1)
        ax.imshow(image, cmap=cmap)
        ax.axis('off')

        if len(feature_points) > 0:
            ax.scatter(feature_points_x[i], feature_points_y[i],
                       color='red', alpha=0.5)

    suptitle = '{} epoch'.format(epoch)
    fig.suptitle(suptitle)
    fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)
