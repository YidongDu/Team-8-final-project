#!/usr/bin/env bash
set -x
DATAPATH="./kitti/"
python save_disp.py --datapath $DATAPATH --testlist ./filenames/test.csv --model gwcnet-g --loadckpt ./checkpoints/kitti15/gwcnet-g/best.ckpt
