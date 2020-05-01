# Group-wise Correlation Network

This code is forked from [xy-guo](https://github.com/xy-guo/GwcNet)

This is the implementation of the paper **Group-wise Correlation Stereo Network**, CVPR 19, Xiaoyang Guo, Kai Yang, Wukui Yang, Xiaogang Wang, and Hongsheng Li
[\[Arxiv\]](https://arxiv.org/)

# How to use

## Environment
* python 3.6
* Pytorch >= 0.4.1

## Data Preparation
Copy the images folders seq_name/image_02 and seq_name/image_03 of the sequence to be evaluated to 'kitti/testing/'

## Pretrained Models
Download the following pretrained weights.
[KITTI 2012/2015](https://drive.google.com/file/d/1fOw2W7CSEzvSKzBAEIIeftxw6CuvH9Hl/view?usp=sharing)

## Evaluation
run the script `./scripts/kittisave.sh` to save png predictions as well as semantic formation of the landmarks to the folder `./predictions`.

# Citation
If you find this code useful in your research, please cite:

```
@inproceedings{guo2019group,
  title={Group-wise Correlation Stereo Network},
  author={Guo, Xiaoyang and Yang, Kai and Yang, Wukui and Wang, Xiaogang and Li, Hongsheng},
  booktitle={Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition},
  pages={3273--3282},
  year={2019}
}
```
