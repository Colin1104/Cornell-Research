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

from env_characterization.srv import *

occ_map = OccupancyGrid()

NUM_CLASSES = 16

IMAGE_SIZE = 18
IMAGE_PIXELS = IMAGE_SIZE**2

tf.logging.set_verbosity(tf.logging.INFO)

def cnn_orientation_model_fn(features, labels, mode):
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
    pool2_flat = tf.reshape(conv2, [-1, IMAGE_PIXELS * 64])
    dense = tf.layers.dense(inputs=pool2_flat, units=1024, activation=tf.nn.relu)
    dropout = tf.layers.dropout(
            inputs=dense, rate=0.2, training=mode == tf.estimator.ModeKeys.TRAIN)
    
    logits = tf.layers.dense(inputs=dropout, units=NUM_CLASSES)
    
    predictions={
            "classes": tf.argmax(input=logits, axis=1),
            "probabilities": tf.nn.softmax(logits, name="softmax_tensor"),
#            "xentropy": tf.losses.softmax_cross_entropy(onehot_labels=logits, logits=logits)
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
    
    complete_data = []
    complete_labels = []
    
    for i in range(1, 2):
        for j in range(NUM_CLASSES if i > 0 else 1):
            sample_data = []
            sample_labels = []
            
            sample_ct = 0
            while True:
                if i == 0:
                    filename = "/home/jonathan/catkin_ws/train_db/" + str(i) + "_" + str(sample_ct) + ".png"
                else:
                    filename = "/home/jonathan/catkin_ws/train_db/" + str(i) + "_" + str(j) + "_" + str(sample_ct) + ".png"
                try:
                    raw_image = mpimg.imread(filename)
                    sample_data.append(np.reshape(raw_image, (IMAGE_PIXELS)))
                    sample_labels.append(j)
                    sample_ct += 1
                except:
                    break
            classed_data.append(np.array(sample_data))
            classed_labels.append(np.array(sample_labels))
    
    #Create the Estimator
    orient_classifier = tf.estimator.Estimator(
            model_fn=cnn_orientation_model_fn, model_dir="/home/jonathan/catkin_ws/train_db/16_orientation_model")
    
#    for i in range(1, 2):
#        train_data = []
#        train_labels = []
#        while True:
#            filename = "/home/jonathan/env_characterization_db/" + str(i) + "_" + str(np.size(train_data, 0)) + ".png"
#            try:
#                raw_image = mpimg.imread(filename)
#                train_data.append(np.reshape(raw_image, (IMAGE_PIXELS)))
#                train_labels.append((np.size(train_labels, 0) - 1)/ 45)
#            except:
#                break
#    
#    #Create the Estimator
#    orient_classifier = tf.estimator.Estimator(
#            model_fn=cnn_orientation_model_fn, model_dir="/home/jonathan/env_characterization_db/8_orientation_model")
    
    # Set up logging for predictions
    # Log the values in the "Softmax" tensor with label "probabilities"
    tensors_to_log = {"probabilities": "softmax_tensor"}
    logging_hook = tf.train.LoggingTensorHook(
            tensors=tensors_to_log, every_n_iter=50)
    
    #Train the model
#    train_data = np.array(train_data)
#    train_labels = np.array(train_labels)
    
    train_data = np.concatenate([classed_data[i] for i in range(np.size(classed_data, 0))])
    train_labels = np.concatenate([classed_labels[i] for i in range(np.size(classed_labels, 0))])
    
    train_batch = [np.array([train_data[i], train_labels[i]]) for i in range(np.size(train_data, 0))]
    
    np.random.shuffle(train_batch)
    
    train_data = np.array([train_batch[i][0] for i in range(np.size(train_batch, 0))])
    train_labels = np.array([train_batch[i][1] for i in range(np.size(train_batch, 0))])
    
    train_input_fn = tf.estimator.inputs.numpy_input_fn(
            x={"x": train_data},
            y=train_labels,
            batch_size=100,
            num_epochs=None,
            shuffle=True)
    orient_classifier.train(
            input_fn=train_input_fn,
            steps=10000,
            hooks=[logging_hook])
            
def evaluate():
    classed_data = []
    classed_labels = []
        
    for i in range(1, 2):
        for j in range(NUM_CLASSES if i > 0 else 1):
            sample_data = []
            sample_labels = []
            
            sample_ct = 0
            while True:
                if i == 0:
                    filename = "/home/jonathan/catkin_ws/train_db/" + str(i) + "_" + str(sample_ct) + ".png"
                else:
                    filename = "/home/jonathan/catkin_ws/train_db/" + str(i) + "_" + str(j) + "_" + str(sample_ct) + ".png"
                try:
                    raw_image = mpimg.imread(filename)
                    sample_data.append(np.reshape(raw_image, (IMAGE_PIXELS)))
                    sample_labels.append(j)
                    sample_ct += 1
                except:
                    break
            classed_data.append(np.array(sample_data))
            classed_labels.append(np.array(sample_labels))
            
    eval_data = np.concatenate([classed_data[i] for i in range(np.size(classed_data, 0))])
    eval_labels = np.concatenate([classed_labels[i] for i in range(np.size(classed_labels, 0))])
    
    #Create the Estimator
    orient_classifier = tf.estimator.Estimator(
            model_fn=cnn_orientation_model_fn, model_dir="/home/jonathan/catkin_ws/train_db/16_orientation_model")
    
    #Evaluate the model and print results
    eval_input_fn = tf.estimator.inputs.numpy_input_fn(
            x={"x": eval_data},
            y=eval_labels,
            num_epochs=1,
            shuffle=False)
    eval_results = orient_classifier.evaluate(input_fn=eval_input_fn)
    print(eval_results)
    
def handle_classify_map(req):
    snippets = req.partitions
    
    orient_classifier = tf.estimator.Estimator(
            model_fn=cnn_orientation_model_fn, model_dir="/home/jonathan/env_characterization_db/8_orientation_model")
    
    print np.size(snippets, 0)
    
    print ("Got Here")
    
    characters = np.array([np.float32(np.array(snippets[i].data) / 100.0 - 0.5) for i in range(0, np.size(snippets, 0))])
    
    pred_input_fn = tf.estimator.inputs.numpy_input_fn(
        x={"x": characters},
        num_epochs=1,
        shuffle=False)
    
    results = orient_classifier.predict(pred_input_fn)
    classes = OccupancyGrid()
    data = []
    features = []
    for idx, p in enumerate(results):
        #print max(p["probabilities"]), p["classes"], idx
        if max(p["probabilities"]) >= 0.7:
            data.append(p["classes"])
            features.append(p["classes"])
        else:
            data.append(0)
    
    #data = [p["classes"] for idx, p in enumerate(results)]
    
    classes.data = data
    print features
    
    return classify_mapResponse(classes)

def main(unused_argv):
#    train()
    evaluate()
    
    rospy.init_node('classifier', anonymous=True)
    rospy.Subscriber("occ_map", OccupancyGrid, callback)
    pub = rospy.Publisher("/classifications", OccupancyGrid, queue_size=10)
    
    s = rospy.Service('classify_map', classify_map, handle_classify_map)
    
    while not rospy.is_shutdown():
        time.sleep(0.1)
    
if __name__ == "__main__":
    tf.app.run()