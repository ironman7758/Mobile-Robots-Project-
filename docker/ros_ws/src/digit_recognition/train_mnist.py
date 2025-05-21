#!/usr/bin/env python3
import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
from digit_recognition.model import CNNModel

def train():
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Training on {device}")

    # 1) MATCH the inference normalization: (x-0.5)/0.5 → data in [-1,1]
    mnist_transform = transforms.Compose([
        transforms.ToTensor(),                 # 0..1
        transforms.Normalize((0.5,), (0.5,))   # → (-1..1)
    ])

    # 2) Load datasets
    train_ds = datasets.MNIST(
        root='data', train=True, download=True,
        transform=mnist_transform
    )
    test_ds = datasets.MNIST(
        root='data', train=False, download=True,
        transform=mnist_transform
    )
    train_loader = DataLoader(train_ds, batch_size=128, shuffle=True, num_workers=4)
    test_loader  = DataLoader(test_ds, batch_size=1000, shuffle=False, num_workers=2)

    # 3) Model, loss, optimizer
    model = CNNModel().to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=1e-3)

    # 4) Training loop
    epochs = 5
    for epoch in range(1, epochs+1):
        model.train()
        total_loss = 0.0
        for imgs, labels in train_loader:
            imgs, labels = imgs.to(device), labels.to(device)
            optimizer.zero_grad()
            outputs = model(imgs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            total_loss += loss.item() * imgs.size(0)
        avg_loss = total_loss / len(train_loader.dataset)
        print(f"Epoch {epoch}/{epochs}, Train Loss: {avg_loss:.4f}")

        # 5) Test accuracy
        model.eval()
        correct = 0
        with torch.no_grad():
            for imgs, labels in test_loader:
                imgs, labels = imgs.to(device), labels.to(device)
                outputs = model(imgs)
                pred = outputs.argmax(dim=1)
                correct += (pred == labels).sum().item()
        acc = correct / len(test_loader.dataset)
        print(f" → Test Accuracy: {acc*100:.2f}%\n")

    # 6) Save weights into your ROS package
    out_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'model', 'mnist_cnn_model.pth'
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    torch.save(model.state_dict(), out_path)
    print("Saved model to", out_path)

if __name__ == '__main__':
    train()
