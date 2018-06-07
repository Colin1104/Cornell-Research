#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Mar  9 12:46:02 2018

@author: jonathan
"""

import tensorflow as tf
import numpy as np
import math
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import random as rand
from random import shuffle
import time

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32MultiArray
from env_characterization.msg import Feature

from env_characterization.srv import *

import cnn_orientation

occ_map = OccupancyGrid()

NUM_CLASSES = 3
NUM_ANGLES = 16

IMAGE_SIZE = 18
IMAGE_PIXELS = IMAGE_SIZE**2

tf.logging.set_verbosity(tf.logging.INFO)

def cnn_model_fn(features, labels, mode):
    # Input layer
    input_layer = tf.reshape(features["x"], [-1, IMAGE_SIZE, IMAGE_SIZE, 1])
    
    #input_layer = tf.image.resize_images(input_layer, [28, 28])
    
    # Convolutional layer #1
    conv1 = tf.layers.conv2d(
            inputs=input_layer,
            filters=32,
            kernel_size=[3, 3],
            padding="same",
            activation=tf.nn.relu)
    
    # Pooling Layer #1
    #pool1 = tf.layers.max_pooling2d(inputs=conv1, pool_size=[2, 2], strides=2)
    
    # Convolutional Layer #2
    conv2 = tf.layers.conv2d(
            inputs=conv1,
            filters=64,
            kernel_size=[3, 3],
            padding="same",
            activation=tf.nn.relu)
            
    #pool2 = tf.layers.max_pooling2d(inputs=conv2, pool_size=[2, 2], strides=2)
    
    #Dense Layer
    pool2_flat = tf.reshape(conv2, [-1, IMAGE_SIZE * IMAGE_SIZE * 64])
    dense = tf.layers.dense(inputs=pool2_flat, units=1024, activation=tf.nn.relu)
    dropout = tf.layers.dropout(
            inputs=dense, rate=0.2, training=mode == tf.estimator.ModeKeys.TRAIN)
    
    logits = tf.layers.dense(inputs=dropout, units=NUM_CLASSES)
    
    predictions={
            "classes": tf.argmax(input=logits, axis=1),
            "probabilities": tf.nn.softmax(logits, name="softmax_tensor")
            }
    
    if mode == tf.estimator.ModeKeys.PREDICT:
        return tf.estimator.EstimatorSpec(mode=mode, predictions=predictions)
    
    #Calculate Loss (for both TRAIN and EVAL modes)
    loss = tf.losses.sparse_softmax_cross_entropy(labels=labels, logits=logits)
    
    # Configure the Training Op (for TRAIN mode)
    if mode == tf.estimator.ModeKeys.TRAIN:
        optimizer = tf.train.GradientDescentOptimizer(learning_rate=0.001)
        train_op = optimizer.minimize(
                loss=loss,
                global_step=tf.train.get_global_step())
        return tf.estimator.EstimatorSpec(mode=mode, loss=loss, train_op=train_op)
    
    #Add evaluation metrics (for EVAL mode)
    eval_metric_ops = {
            "accuracy": tf.metrics.accuracy(
                    labels=labels, predictions=predictions["classes"])}
    return tf.estimator.EstimatorSpec(
            mode=mode, loss=loss, eval_metric_ops=eval_metric_ops)
    
def callback(data):
    global occ_map
    occ_map = data
    
def train():
    #Assemble datasets
    classed_data = []
    classed_labels = []
    
    for i in range(NUM_CLASSES):
        sample_data = []
        sample_labels = []
        for j in range(NUM_ANGLES if i == 1 else 1):
            sample_ct = 0
            while True:
                if i != 1:
                    filename = "/home/jonathan/catkin_ws/train_db/" + str(i) + "_" + str(sample_ct) + ".png"
                else:
                    filename = "/home/jonathan/catkin_ws/train_db/" + str(i) + "_" + str(j) + "_" + str(sample_ct) + ".png"
                try:
                    raw_image = mpimg.imread(filename)
                    sample_data.append(np.reshape(raw_image, (IMAGE_PIXELS)))
                    sample_labels.append(i)
                    sample_ct += 1
                except:
                    break
        classed_data.append(np.array(sample_data))
        classed_labels.append(np.array(sample_labels))
    
    #Create the Estimator
    mnist_classifier = tf.estimator.Estimator(
            model_fn=cnn_model_fn, model_dir="/home/jonathan/Cornell-Research/env_classifier_model")
    
    # Set up logging for predictions
    # Log the values in the "Softmax" tensor with label "probabilities"
    tensors_to_log = {"probabilities": "softmax_tensor"}
    logging_hook = tf.train.LoggingTensorHook(
            tensors=tensors_to_log, every_n_iter=50)
    
    #Train the model
    sample_counts = [np.size(classed_data[i], 0) for i in range(NUM_CLASSES)]
    class_batch = min(sample_counts)
    max_sample = max(sample_counts)

#    for i in range(NUM_CLASSES):
#        np.random.shuffle(classed_data[i])
#        
#        class_size = np.size(classed_data[i], 0)
#        resized_class = classed_data[i][:]
#        for j in range(max_sample / class_size - 1):
#            resized_class = np.append(resized_class, classed_data[i], 0)
#        
#        resized_class = np.append(resized_class, classed_data[i][0:(max_sample % class_size)], 0)
#        classed_data[i] = resized_class[:]
#        classed_labels[i] = np.ones(max_sample, dtype="int32") * i
    
    #Extend each class set to be even multiple of class_batch,
    #using repeated data from beginning of set
#    for i in range(NUM_CLASSES):
#        classed_data[i] = np.append(classed_data[i], classed_data[i][0:(class_batch - np.size(classed_data[i], 0) % class_batch)], 0)
        
    N = max(sample_counts) / class_batch
    
    print sample_counts, N
    
    train_data = np.concatenate([classed_data[i] for i in range(np.size(classed_data, 0))])
    train_labels = np.concatenate([classed_labels[i] for i in range(np.size(classed_labels, 0))])
    
    train_batch = [np.array([train_data[i], train_labels[i]]) for i in range(np.size(train_data, 0))]
    
    np.random.shuffle(train_batch)
    
    train_data = np.array([train_batch[i][0] for i in range(np.size(train_batch, 0))])
    train_labels = np.array([train_batch[i][1] for i in range(np.size(train_batch, 0))])
    
#    train_data = np.array(classed_data[1])
#    train_labels = np.array(classed_labels[1])
    
    
    train_input_fn = tf.estimator.inputs.numpy_input_fn(
            x={"x": train_data},
            y=train_labels,
            batch_size=100,
            num_epochs=None,
            shuffle=True)
    mnist_classifier.train(
            input_fn=train_input_fn,
            steps=60000,
            hooks=[logging_hook])
                
def evaluate():
    classed_data = []
    classed_labels = []
    
    for i in range(NUM_CLASSES):
        sample_data = []
        sample_labels = []
        for j in range(NUM_ANGLES if i == 1 else 1):
            sample_ct = 0
            while True:
                if i != 1:
                    filename = "/home/jonathan/catkin_ws/train_db/" + str(i) + "_" + str(sample_ct) + ".png"
                else:
                    filename = "/home/jonathan/catkin_ws/train_db/" + str(i) + "_" + str(j) + "_" + str(sample_ct) + ".png"
                try:
                    raw_image = mpimg.imread(filename)
                    sample_data.append(np.reshape(raw_image, (IMAGE_PIXELS)))
                    sample_labels.append(i)
                    sample_ct += 1
                    if i > 0:
                        complete_data.append(np.reshape(raw_image, (IMAGE_PIXELS)))
                        complete_labels.append(i)
                except:
                    break
        classed_data.append(np.array(sample_data))
        classed_labels.append(np.array(sample_labels))

    #Convert lists to arrays for numpy input fn
    eval_data = np.array(classed_data[0])
    eval_labels = np.array(classed_labels[0])
    
    #Create the Estimator
    mnist_classifier = tf.estimator.Estimator(
            model_fn=cnn_model_fn, model_dir="/home/jonathan/catkin_ws/env_classifier_model_gpu")
    
    #Evaluate the model and print results
    eval_input_fn = tf.estimator.inputs.numpy_input_fn(
            x={"x": eval_data},
            y=eval_labels,
            num_epochs=1,
            shuffle=False)
    eval_results = mnist_classifier.evaluate(input_fn=eval_input_fn)
    print(eval_results)
    
def handle_classify_map(req):
    snippets = req.partitions
    
    mnist_classifier = tf.estimator.Estimator(
            model_fn=cnn_model_fn, model_dir="/home/jonathan/catkin_ws/env_classifier_model_gpu")
            
    orient_classifier = tf.estimator.Estimator(
            model_fn=cnn_orientation.cnn_orientation_model_fn, model_dir="/home/jonathan/catkin_ws/train_db_current_experiment/16_orientation_model")
    
    print np.size(snippets, 0)
    
    print ("Got Here")
    
    characters = np.array([np.float32(np.array(snippets[i].data)) for i in range(0, np.size(snippets, 0))])
    
    np.set_printoptions(precision=1)
    print ("Input Data")
    print np.reshape(characters[0], (IMAGE_SIZE, IMAGE_SIZE))
    
    print ("Reading from Img")
    img = mpimg.imread("/home/jonathan/catkin_ws/snippet_img.png")    
    print (img)
    
    pred_input_fn = tf.estimator.inputs.numpy_input_fn(
        x={"x": characters},
        num_epochs=1,
        shuffle=False)
    
    results = mnist_classifier.predict(pred_input_fn)
    classes = OccupancyGrid()
    data = []
    features = []
    feature_snips = []
    feature_ct = 0
    for idx, p in enumerate(results):
        #print max(p["probabilities"]), p["classes"], idx
        feat = Feature()
        if max(p["probabilities"]) > 0.8 and p["classes"] > 0:
            feat.feature = p["classes"]
            features.append(p["classes"])
            if p["classes"] == 1:
                feature_snips.append(characters[idx])
            feature_ct += 1
        else:
            feat.feature = -1
        data.append(feat)

    print ("Found %s features" % feature_ct)
    print np.size(feature_snips, 0), np.size(feature_snips, 1)

    feature_snips = np.array(feature_snips)    
    
    #Classify orientation of feature
    orient_input_fn = tf.estimator.inputs.numpy_input_fn(
        x={"x": feature_snips},
        num_epochs=1,
        shuffle=False)
        
    orient_results = orient_classifier.predict(orient_input_fn)
    
    orients = []
    for jdx, q in enumerate(orient_results):
        orients.append(q["classes"])
        
    print("Found params")

    feature_count = 0
    for i in range(np.size(data, 0)):
        if data[i].feature == 1:
            data[i].param = orients[feature_count]
            feature_count += 1
    print orients
    
#    feature_count = 0
#    for i in range(np.size(data, 0)):
#        if data[i] > 0 and feature_count < np.size(orients, 0):
#            data[i] = orients[feature_count]
#            feature_count += 1
#    print orients
    
    #data = [p["classes"] for idx, p in enumerate(results)]
    
    #classes.characters = data
    
    return classify_mapResponse(data)

def main(unused_argv):
#    train()
#    evaluate()
    
    rospy.init_node('classifier', anonymous=True)
    rospy.Subscriber("occ_map", OccupancyGrid, callback)
    pub = rospy.Publisher("/classifications", OccupancyGrid, queue_size=10)
    
    s = rospy.Service('classify_map', classify_map, handle_classify_map)
    
    while not rospy.is_shutdown():
        time.sleep(0.1)
    
if __name__ == "__main__":
    tf.app.run()