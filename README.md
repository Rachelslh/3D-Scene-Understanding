# 3D Scene Understanding
Understanding the image is fundamental to the atificial vision. The first approaches were centered
on questions relating to the observed scene "What is present" and "Where". Following this methodology,
the scenes are recognized by detecting the objects in the scene as well as their positions.
This work concerns depth images and takes as input 3D format data (point cloud) using Kinect.
The proposed approach uses fundamental and basic algorithms like Random Sample Concensus (RANSAC) to get the requested output and applies Guzman's work in the field of Polyhedra Objects Classification in 3D.

Modules : 
1. Acquisition and Preprocessing
2. Planar Segmentation using RANSAC
3. Junction Detection
4. Edge Detection
5. Semantic labeling of Edges
6. Semantic labeling of Nodes (A.Guzman)
7. Classification using an MLP structure (Machine Learning)

## Classes
1, 2, 5 ,and 7 octants

## Dataset
Made our own dataset of octants, manually labelled. Can be downloaded [here](https://drive.google.com/drive/folders/1T7yTTtDLASLWzn-XHshiRxWtCfAXRAYx?usp=sharing).
## Accuracy
Using one hidden layer with a Sigmoid activation function and an output layer with a Softmax function, this neural network is accurate at 89%.

![alt text](https://github.com/Rachelslh/3D_Scene_Understanding/blob/master/nn_results.png?raw=true)

## Perspectives
This work is only applied on polyedra objects composed of octants and can be further extended to real life objects.


