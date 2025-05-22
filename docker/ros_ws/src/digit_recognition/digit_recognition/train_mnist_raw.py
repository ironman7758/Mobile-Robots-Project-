#!/usr/bin/env python3
import os
import struct
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset, random_split
from ament_index_python.packages import get_package_share_directory
from digit_recognition.model import CNNModel

#
# 1) Raw MNIST IDX loader
#
class MnistDataloader:
    def __init__(self, data_dir):
        self.train_images_fp = os.path.join(data_dir, 'train-images-idx3-ubyte')
        self.train_labels_fp = os.path.join(data_dir, 'train-labels-idx1-ubyte')
        self.test_images_fp  = os.path.join(data_dir, 't10k-images-idx3-ubyte')
        self.test_labels_fp  = os.path.join(data_dir, 't10k-labels-idx1-ubyte')

    def read_images_labels(self, img_fp, lbl_fp):
        # load labels
        with open(lbl_fp, 'rb') as f:
            magic, size = struct.unpack(">II", f.read(8))
            assert magic == 2049, f"Bad magic in {lbl_fp}"
            labels = np.frombuffer(f.read(), dtype=np.uint8)

        # load images
        with open(img_fp, 'rb') as f:
            magic, size2, rows, cols = struct.unpack(">IIII", f.read(16))
            assert magic == 2051, f"Bad magic in {img_fp}"
            data = np.frombuffer(f.read(), dtype=np.uint8)
        images = data.reshape(size2, rows, cols)

        return images, labels

    def load_data(self):
        x_train, y_train = self.read_images_labels(self.train_images_fp,
                                                   self.train_labels_fp)
        x_test,  y_test  = self.read_images_labels(self.test_images_fp,
                                                   self.test_labels_fp)
        return (x_train, y_train), (x_test, y_test)


def main():
    # 2) Hyperparameters
    batch_size      = 64
    test_batch_size = 1000
    epochs          = 10
    lr              = 1e-3
    device          = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # 3) Locate raw files under your ROS package
    share_dir = get_package_share_directory('digit_recognition')
    data_dir  = os.path.join(share_dir, 'data', 'MNIST')  # adjust if needed

    loader = MnistDataloader(data_dir)
    (x_train, y_train), (x_test, y_test) = loader.load_data()

    # 4) Normalize & convert to tensors
    #    MNIST pixels are 0-255; we want floats in [-1,1]
    x_train = (torch.from_numpy(x_train).unsqueeze(1).float() / 255.0 - 0.5) / 0.5
    y_train = torch.from_numpy(y_train).long()
    x_test  = (torch.from_numpy(x_test).unsqueeze(1).float() / 255.0 - 0.5) / 0.5
    y_test  = torch.from_numpy(y_test).long()

    # 5) Create datasets & loaders
    full_train_ds = TensorDataset(x_train, y_train)
    # if you want a validation split:
    # train_ds, val_ds = random_split(full_train_ds, [55000, 5000])
    train_loader = DataLoader(full_train_ds, batch_size=batch_size, shuffle=True)
    test_loader  = DataLoader(TensorDataset(x_test, y_test),
                              batch_size=test_batch_size, shuffle=False)

    # 6) Model, loss, optimizer
    model     = CNNModel().to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)

    # 7) Training loop
    for epoch in range(1, epochs+1):
        model.train()
        tot_loss = 0.0
        for batch_idx, (data, target) in enumerate(train_loader, start=1):
            data, target = data.to(device), target.to(device)
            optimizer.zero_grad()
            output = model(data)
            loss   = criterion(output, target)
            loss.backward()
            optimizer.step()
            tot_loss += loss.item()

            if batch_idx % 100 == 0:
                print(f"Epoch {epoch} [{batch_idx*len(data)}/{len(train_loader.dataset)}] "
                      f"Loss: {tot_loss/batch_idx:.4f}")

        # Validation
        model.eval()
        correct = 0
        with torch.no_grad():
            for data, target in test_loader:
                data, target = data.to(device), target.to(device)
                out = model(data)
                pred = out.argmax(dim=1)
                correct += pred.eq(target).sum().item()
        acc = correct / len(test_loader.dataset)
        print(f"Epoch {epoch} complete: Test Accuracy: {acc*100:.2f}%\n")

    # 8) Save weights
    save_dir = os.path.join(share_dir, 'model')
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, 'mnist_cnn_model.pth')
    torch.save(model.state_dict(), save_path)
    print(f"Model weights saved to: {save_path}")


if __name__ == '__main__':
    main()
