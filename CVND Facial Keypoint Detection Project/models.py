## TODO: define the convolutional neural network architecture

import torch
import torch.nn as nn
import torch.nn.functional as F
# can use the below import should you choose to initialize the weights of your Net
import torch.nn.init as I


class Net(nn.Module):

    def __init__(self):
        super(Net, self).__init__()
        
        self.conv2D_1 = nn.Conv2d(1, 32, 5)
        
        self.conv2D_2 = nn.Conv2d(32, 48, 5)
        
        self.conv2D_3 = nn.Conv2d(48, 64, 5)
        
        self.conv2D_4 = nn.Conv2d(64, 96, 5)
        
        self.batchnorm_1 = nn.BatchNorm2d(32)
        
        self.batchnorm_2 = nn.BatchNorm2d(48)
        
        self.batchnorm_3 = nn.BatchNorm2d(64)
        
        self.batchnorm_4 = nn.BatchNorm2d(96)
        
        self.batchnorm_1D_1 = nn.BatchNorm1d(2400)
        
        self.batchnorm_1D_2 = nn.BatchNorm1d(1024)
        
        self.dropout_2D_1 = nn.Dropout2d(p=0.1)
        
        self.dropout_2D_2 = nn.Dropout2d(p=0.3)
        
        # maxpool layer
        # pool with kernel_size=2, stride=2
        self.pool = nn.MaxPool2d(2, 2)
        
        self.fc1_dropout = nn.Dropout(p=0.3)
        
        self.fc2_dropout = nn.Dropout(p=0.4)
        
        self.fc1 = nn.Linear(9600, 2400) 
        
        self.fc2 = nn.Linear(2400, 1024)
        
        ## output 136 values, 2 for each of the 68 keypoint (x, y) pairs
        self.fc_final = nn.Linear(1024, 136)
        
    def forward(self, x):
        # 4 conv/elu + pool layers and a dropout
        x = self.pool(F.elu(self.conv2D_1(x)))
        x = self.batchnorm_1(x)
        x = self.dropout_2D_1(x)
        x = self.pool(F.elu(self.conv2D_2(x)))
        x = self.batchnorm_2(x)
        x = self.dropout_2D_1(x)
        x = self.pool(F.elu(self.conv2D_3(x)))
        x = self.batchnorm_3(x)
        x = self.dropout_2D_2(x)
        x = self.pool(F.elu(self.conv2D_4(x)))
        x = self.batchnorm_4(x)
        x = self.dropout_2D_2(x)
        

        # this line of code is the equivalent of Flatten in Keras
        x = x.view(x.size(0), -1)
        
        
        # two linear layers with dropout in between
        x = F.elu(self.fc1(x))
        x = self.batchnorm_1D_1(x)
        x = self.fc1_dropout(x)
        x = F.relu(self.fc2(x))
        x = self.batchnorm_1D_2(x)
        x = self.fc2_dropout(x)
        # final output
        x = self.fc_final(x)
        
        return x
    
    
    
   
