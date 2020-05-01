Author: Roger(Aohan) Mei
Date: 04/30/2019
1. With image files incorporated correctly, you should first treat the whole dataset as the train_set and run the FAST_Feature.m to construct the BRIEF descriptor.
2. Save the BRIEF descriptor as a .mat file and use the function grow_tree_kmean_indexed.m to constuct the tree.
3. run the Loop_Closure_detection.m to obtain the loop closure pairs.
