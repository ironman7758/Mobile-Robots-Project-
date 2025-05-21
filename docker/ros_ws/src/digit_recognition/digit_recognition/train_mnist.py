#!/usr/bin/env python3
import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Subset
from torchvision import datasets, transforms
from ament_index_python.packages import get_package_share_directory
from digit_recognition.model import CNNModel

def main():
    # 1) Hyperparameters
    batch_size      = 64
    test_batch_size = 1000
    epochs          = 10
    lr              = 1e-3
    use_cuda        = torch.cuda.is_available()
    device          = torch.device("cuda" if use_cuda else "cpu")

    # 2) Transforms
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.5,), (0.5,))
    ])

    # 3) Load full MNIST then subset to first 10 000 samples
    full_train = datasets.MNIST(
        root=os.path.expanduser('~/.mnist'),
        train=True,
        download=True,
        transform=transform
    )
    subset_indices = list(range(10000))
    train_ds = Subset(full_train, subset_indices)

    test_ds = datasets.MNIST(
        root=os.path.expanduser('~/.mnist'),
        train=False,
        download=True,
        transform=transform
    )

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    test_loader  = DataLoader(test_ds, batch_size=test_batch_size, shuffle=False)

    # 4) Model, loss, optimizer
    model     = CNNModel().to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)

    # 5) Training loop
    for epoch in range(1, epochs+1):
        model.train()
        total_loss = 0
        for batch_idx, (data, target) in enumerate(train_loader, 1):
            data, target = data.to(device), target.to(device)
            optimizer.zero_grad()
            output = model(data)
            loss   = criterion(output, target)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()
            if batch_idx % 100 == 0:
                print(f"Epoch {epoch} [{batch_idx*len(data)}/{len(train_loader.dataset)}]"
                      f" Loss: {total_loss/batch_idx:.4f}")

        # 6) Validation
        model.eval()
        correct = 0
        with torch.no_grad():
            for data, target in test_loader:
                data, target = data.to(device), target.to(device)
                output = model(data)
                pred   = output.argmax(dim=1)
                correct += pred.eq(target).sum().item()
        acc = correct / len(test_loader.dataset)
        print(f"Epoch {epoch} complete: Test accuracy: {acc*100:.2f}%\n")

    # 7) Save trained weights
    share_dir = get_package_share_directory('digit_recognition')
    save_path = os.path.join(share_dir, 'model', 'mnist_cnn_model.pth')
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    torch.save(model.state_dict(), save_path)
    print(f"Model weights saved to: {save_path}")

if __name__ == '__main__':
    main()
