# YOLOv3-Kitti-Object-Detector
Minimal implementation of YOLOv3 in PyTorch.
And Training from Kitti dataset

## HI THERE~ THIS repo is forked from [packyan](https://github.com/packyan/PyTorch-YOLOv3-kitti)

## Installation
    $ cd Object_Detection/
    $ sudo pip3 install -r requirements.txt


##### Download pretrained weights
The pretrained weights for KITTI dataset for test or detect, can be downloaded from [pretrained weights file](https://drive.google.com/file/d/1BRJDDCMRXdQdQs6-x-3PmlzcEuT9wxJV/view?usp=sharing). Put it into `weights` folder, the path:
`weights/kitti.weights`

##### Prepare Data
Copy all the image files from seq_name/image_02/data into data/samples.

Example. seq_name = "2011_09_26/2011_09_26_drive_0005_sync/image_02/data"

## Detection
Uses pretrained weights to make predictions on images. `weights/kitti.weights` was trained by kitti data set.
`python3 detect.py --image_folder /data/samples`

The output images with bounding boxes and csv files with semantic information is saved in the 'output' directory.

The csv files data is stored in the following format:

x_centroid_pixel|y_centroid_pixel|bbox_width|bbox_height|class_probability|class_number|class_0_prob|class_1_prob|class_2_prob|class_3_prob|class_4_prob|class_5_prob|class_6_prob|class_7_prob

```
## Paper
### YOLOv3: An Incremental Improvement
_Joseph Redmon, Ali Farhadi_ <br>

**Abstract** <br>
We present some updates to YOLO! We made a bunch
of little design changes to make it better. We also trained
this new network that’s pretty swell. It’s a little bigger than
last time but more accurate. It’s still fast though, don’t
worry. At 320 × 320 YOLOv3 runs in 22 ms at 28.2 mAP,
as accurate as SSD but three times faster. When we look
at the old .5 IOU mAP detection metric YOLOv3 is quite
good. It achieves 57.9 AP50 in 51 ms on a Titan X, compared
to 57.5 AP50 in 198 ms by RetinaNet, similar performance
but 3.8× faster. As always, all the code is online at
https://pjreddie.com/yolo/.

[[Paper]](https://pjreddie.com/media/files/papers/YOLOv3.pdf) [[Original Implementation]](https://github.com/pjreddie/darknet)


## Credit
```
@article{yolov3,
  title={YOLOv3: An Incremental Improvement},
  author={Redmon, Joseph and Farhadi, Ali},
  journal = {arXiv},
  year={2018}
}
```
