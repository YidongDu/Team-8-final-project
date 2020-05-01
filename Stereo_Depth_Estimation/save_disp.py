from __future__ import print_function, division
import argparse
import os
import torch
import torch.nn as nn
import torch.nn.parallel
import torch.backends.cudnn as cudnn
import torch.optim as optim
import torch.utils.data
from torch.autograd import Variable
import torchvision.utils as vutils
import torch.nn.functional as F
import numpy as np
import time
from tensorboardX import SummaryWriter
from datasets import __datasets__
from models import __models__
from utils import *
from torch.utils.data import DataLoader
import gc
from skimage import io
import cv2
from os import listdir
from os.path import isfile, join
import csv 

cudnn.benchmark = True

parser = argparse.ArgumentParser(description='Group-wise Correlation Stereo Network (GwcNet)')
parser.add_argument('--model', default='gwcnet-g', help='select a model structure', choices=__models__.keys())
parser.add_argument('--maxdisp', type=int, default=192, help='maximum disparity')

parser.add_argument('--dataset', default='kitti', help='dataset name', choices=__datasets__.keys())
parser.add_argument('--datapath', required=True, help='data path')
parser.add_argument('--testlist', required=True, help='testing list')
parser.add_argument('--loadckpt', required=True, help='load the weights from a specific checkpoint')

# parse arguments
args = parser.parse_args()

# dataset, dataloader
StereoDataset = __datasets__[args.dataset]
test_dataset = StereoDataset(args.datapath, args.testlist, False)
TestImgLoader = DataLoader(test_dataset, 1, shuffle=False, num_workers=4, drop_last=False)

# model, optimizer
model = __models__[args.model](args.maxdisp)
model = nn.DataParallel(model)
model.cuda()

# load parameters
print("loading model {}".format(args.loadckpt))
state_dict = torch.load(args.loadckpt)
model.load_state_dict(state_dict['model'])


def test():
    os.makedirs('./predictions', exist_ok=True)
    baseline = 0.5407
    focal = 707.09
    P = np.array([[7.070912e+02, 0.000000e+00, 6.018873e+02, 4.688783e+01],[ 0.000000e+00, 7.070912e+02, 1.831104e+02, 1.178601e-01], [0.000000e+00, 0.000000e+00, 1.000000e+00, 6.203223e-03]])
    for batch_idx, sample in enumerate(TestImgLoader):
        start_time = time.time()
        disp_est_np = tensor2numpy(test_sample(sample))
        top_pad_np = tensor2numpy(sample["top_pad"])
        right_pad_np = tensor2numpy(sample["right_pad"])
        left_filenames = sample["left_filename"]
        print('Iter {}/{}, time = {:3f}'.format(batch_idx, len(TestImgLoader),
                                                time.time() - start_time))

        for disp_est, top_pad, right_pad, fn in zip(disp_est_np, top_pad_np, right_pad_np, left_filenames):
            assert len(disp_est.shape) == 2
            disp_est = np.array(disp_est[top_pad:, :-right_pad], dtype=np.float32)
            fn = os.path.join("predictions_test", fn.split('/')[-1])
#             print("saving to", fn, disp_est.shape)
            data = []
            xyz = []
            with open("../PyTorch-YOLOv3-kitti/output/%s.csv"%(fn.split('/')[-1].split('.')[0])) as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for row in csv_reader:
                    data.append(row)
                    depth = focal * baseline / disp_est[int(float(row[1]))][int(float(row[0]))]
#                     print(depth)
                    x = np.array([int(float(row[0])), int(float(row[1])), 1])
                    y1 = np.dot(np.linalg.pinv(P),x)
                    xyz.append([y1[0]*depth/y1[2], -y1[1]*depth/y1[2], depth])
            with open("predictions_test/%s.csv"%(fn.split('/')[-1].split('.')[0]), mode='w') as csv_file2:
                csv_writer = csv.writer(csv_file2, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                for row,pos in zip(data,xyz):
                    csv_writer.writerow([row[0],row[1],row[2],row[3],row[4],row[5],row[6],row[7],row[8],row[9],row[10],row[11],row[12],row[13], row[14],pos[0],pos[1],pos[2]])
                    
            disp_est_uint = np.round(disp_est * 256).astype(np.uint16)
            #disp_est_uint = cv2.rectangle(disp_est_uint, (667, 173), (741, 235), (255,255,255), -1)
            cv2.imwrite(fn, disp_est_uint)


# test one sample
@make_nograd_func
def test_sample(sample):
    model.eval()
    disp_ests = model(sample['left'].cuda(), sample['right'].cuda())
    return disp_ests[-1]


if __name__ == '__main__':
    test()
