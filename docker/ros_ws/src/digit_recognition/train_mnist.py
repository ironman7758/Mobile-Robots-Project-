#!/usr/bin/env python3
import os
import torch
from torch import nn, optim
from torchvision import datasets, transforms
from torch.utils.data import DataLoader
from digit_recognition.model import CNNModel

def main():
    batch_size, epochs, lr = 64, 5, 1e-3
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.5,), (0.5,))
    ])
    train_ds = datasets.MNIST('data', train=True,  download=True, transform=transform)
    test_ds  = datasets.MNIST('data', train=False, download=True, transform=transform)
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    test_loader  = DataLoader(test_ds,  batch_size=1000)

    model     = CNNModel().to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)

    for epoch in range(1, epochs+1):
        model.train()
        total_loss = 0
        for imgs, labels in train_loader:
            imgs, labels = imgs.to(device), labels.to(device)
            optimizer.zero_grad()
            out = model(imgs)
            loss = criterion(out, labels)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()
        print(f"Epoch {epoch}/{epochs}  Loss: {total_loss/len(train_loader):.4f}")

    model.eval()
    correct = total = 0
    with torch.no_grad():
        for imgs, labels in test_loader:
            imgs, labels = imgs.to(device), labels.to(device)
            out = model(imgs)
            _, preds = torch.max(out, 1)
            correct += (preds == labels).sum().item()
            total   += labels.size(0)
    print(f"Test Accuracy: {100 * correct/total:.2f}%")

    os.makedirs('model', exist_ok=True)
    torch.save(model.state_dict(), 'model/mnist_cnn_model.pth')
    print("Saved model to model/mnist_cnn_model.pth")

if __name__ == '__main__':
    main()
