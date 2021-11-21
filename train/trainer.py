import torch
import wandb
import time
import os

import sys
sys.path.append('.')
from util.print_progress_bar import print_progress_bar


class Tranier():
    def __init__(self,
        train_loader: torch.utils.data.DataLoader,
        valid_loader: torch.utils.data.DataLoader,
        model: torch.nn.Module,
        calc_loss: callable,
        optimizer: torch.optim.Optimizer,
        out_dir: str,
        wandb_flag: bool = False,
        gpu_num: list = [0],
        early_stopping_count: int = 1000,
    ):
        self.model = model
        self.train_loader = train_loader
        self.valid_loader = valid_loader
        self.calc_loss = calc_loss
        self.optimizer = optimizer
        self.out_dir = out_dir
        self.wandb_flag = wandb_flag
        self.early_stopping_count = early_stopping_count

        # device setting
        self.device = torch.device(f'cuda:{gpu_num[0]}'
            if torch.cuda.is_available() else 'cpu')
        print('device:', self.device)
        if torch.cuda.device_count() > 1 and len(gpu_num) > 1:
            print('Let\'s use', torch.cuda.device_count(), 'GPUs!')
            model = torch.nn.DataParallel(model, device_ids=gpu_num)
        model.to(self.device)

        # acceleration
        self.scaler = torch.cuda.amp.GradScaler()
        torch.backends.cudnn.benchmark = True

        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
        print(f'save to {out_dir}')

    def train(self, n_epochs: int, callback: callable(int) = None):
        for epoch in range(n_epochs + 1):
            start = time.time()

            # train
            running_loss = 0.0
            self.model.train()
            for i, batch in enumerate(self.train_loader):
                with torch.cuda.amp.autocast():
                    loss = self.calc_loss(batch)

                self.optimizer.zero_grad()
                self.scaler.scale(loss).backward()
                self.scaler.step(self.optimizer)
                self.scaler.update()

                running_loss += loss.item()

                header = 'epoch: {}'.format(epoch)
                print_progress_bar(
                    i, len(self.train_loader), end='', header=header)
            train_loss = running_loss / len(self.train_loader)
            self.train_losses.append(train_loss)

            # valid
            running_loss = 0.0
            self.model.eval()
            for batch in self.valid_loader:
                with torch.zero_grad():
                    loss = self.calc_loss(batch)
                running_loss += loss.item()
            valid_loss = running_loss / len(self.valid_loader)
            self.valid_losses.append(valid_loss)

            # log
            end = time.time()
            elapsed_time = end - start
            self.total_elapsed_time += elapsed_time
            log = '\r\033[K' + 'epoch: {}'.format(epoch)
            log += '  train loss: {:.6f}'.format( train_loss)
            log += '  valid loss: {:.6f}'.format(valid_loss)
            log += '  elapsed time: {:.3f}'.format(elapsed_time)
            log += '  early stopping: {}'.format(early_stopping_counter)
            print(log)

            # save model
            if epoch % 100 == 0:
                encoder_param_dir = os.path.join(self.out_dir, 'model_param')
                if not os.path.exists(encoder_param_dir):
                    os.mkdir(encoder_param_dir)
                path_encoder_param = os.path.join(
                    encoder_param_dir,
                    'model_param_{:06d}.pt'.format(epoch))
                torch.save(self.model.state_dict(), path_encoder_param)

                if self.wandb_flag:
                    wandb.save(path_encoder_param)

            # save checkpoint
            path_checkpoint = os.path.join(self.out_dir, 'checkpoint.pt')
            torch.save({
                'epoch': epoch,
                'model_state_dict': self.model.state_dict(),
                'optimizer_state_dict': self.optimizer.state_dict(),
                'loss': loss,
            }, path_checkpoint)

            # wandb
            if self.wandb_flag:
                wandb.log({
                    'epoch': epoch,
                    'iteration': len(self.train_loader) * epoch,
                    'train_loss': train_loss,
                    'valid_loss': valid_loss,
                })
                wandb.save(path_checkpoint)

            if valid_loss < best_test:
                best_test = valid_loss
                early_stopping_counter = 0

                # save model
                path_model_param_best = os.path.join(
                    self.out_dir, 'model_param_best.pt')
                torch.save(self.model.state_dict(), path_model_param_best)
                if self.wandb_flag:
                    wandb.save(path_model_param_best)

            else:
                # Early Stopping
                early_stopping_counter += 1
                if early_stopping_counter >= self.early_stopping_count:
                    print('Early Stopping!')
                    break

            callback(epoch)

        print('total elapsed time: {} [s]'.format(self.total_elapsed_time))
