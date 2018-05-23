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

NUM_CLASSES = 2
NUM_EXAMPLES = 360

IMAGE_SIZE = 12
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
    pool2_flat = tf.reshape(conv2, [-1, 12 * 12 * 64])
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
    
    complete_data = []
    complete_labels = []
    
    for i in range(NUM_CLASSES):
        sample_data = []
        sample_labels = []
        while True:
            filename = "/home/jonathan/env_characterization_db/" + str(i) + "_" + str(np.size(sample_data, 0)) + ".png"
            try:
                raw_image = mpimg.imread(filename)
                sample_data.append(np.reshape(raw_image, (IMAGE_PIXELS)))
                sample_labels.append(i)
                if i > 0:
                    complete_data.append(np.reshape(raw_image, (IMAGE_PIXELS)))
                    complete_labels.append(i)
            except:
                break
        classed_data.append(np.array(sample_data))
        classed_labels.append(np.array(sample_labels))
    
    #Create the Estimator
    mnist_classifier = tf.estimator.Estimator(
            model_fn=cnn_model_fn, model_dir="/home/jonathan/env_characterization_db/env_classifier_model")
    
    # Set up logging for predictions
    # Log the values in the "Softmax" tensor with label "probabilities"
    tensors_to_log = {"probabilities": "softmax_tensor"}
    logging_hook = tf.train.LoggingTensorHook(
            tensors=tensors_to_log, every_n_iter=50)
    
    #Train the model
    sample_counts = [np.size(classed_data[i], 0) for i in range(NUM_CLASSES)]
    class_batch = min(sample_counts)
    N = max(sample_counts) / class_batch
    
    print sample_counts, N
    
    TOTAL_STEPS = 10000
    negative_class = classed_data[0][:]
#    rand.shuffle(negative_class)
    
    for i in range (1):
        train_data = complete_data[:]
        train_labels = complete_labels[:]
#        train_data.extend(negative_class[i * class_batch:(i + 1) * class_batch])
#        train_labels.extend(np.zeros(class_batch, dtype=int))
        train_data.extend(negative_class[0:360])
        train_labels.extend(np.zeros(360, dtype=int))
        
        train_data = np.array(train_data)
        train_labels = np.array(train_labels)
        
        train_input_fn = tf.estimator.inputs.numpy_input_fn(
                x={"x": train_data},
                y=train_labels,
                batch_size=100,
                num_epochs=1000,
                shuffle=True)
        mnist_classifier.train(
                input_fn=train_input_fn,
                steps=None,
                hooks=[logging_hook])
                
def evaluate():
    eval_data = []
    eval_labels = []

#    for i in range(65 * 45):
#        filename_test = "/home/jonathan/env_characterization_db/0_" + str(i) + ".png"
#    
#        raw_image = mpimg.imread(filename_test)
#        eval_data.append(np.reshape(raw_image, (IMAGE_PIXELS)))
#        eval_labels.append(0)
        
    for i in range(360):
        filename_test = "/home/jonathan/env_characterization_db/1_" + str(i) + ".png"
    
        raw_image = mpimg.imread(filename_test)
        eval_data.append(np.reshape(raw_image, (IMAGE_PIXELS)))
        eval_labels.append(1)
        
    #Convert lists to arrays for numpy input fn
    eval_data = np.array(eval_data)
    eval_labels = np.array(eval_labels)
    
    #Create the Estimator
    mnist_classifier = tf.estimator.Estimator(
            model_fn=cnn_model_fn, model_dir="/home/jonathan/env_characterization_db/env_classifier_model")
    
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
            model_fn=cnn_model_fn, model_dir="/home/jonathan/env_characterization_db/env_classifier_model")
            
    orient_classifier = tf.estimator.Estimator(
            model_fn=cnn_orientation.cnn_orientation_model_fn, model_dir="/home/jonathan/env_characterization_db/8_orientation_model")
    
    print np.size(snippets, 0)
    
    print ("Got Here")
    
    characters = np.array([np.float32(np.array(snippets[i].data) / 100.0 - 0.5) for i in range(0, np.size(snippets, 0))])
    
    pred_input_fn = tf.estimator.inputs.numpy_input_fn(
        x={"x": characters},
        num_epochs=1,
        shuffle=False)
    
    results = mnist_classifier.predict(pred_input_fn)
    classes = OccupancyGrid()
    data = []
    features = []
    feature_snips = []
    for idx, p in enumerate(results):
        #print max(p["probabilities"]), p["classes"], idx
        feat = Feature()
        if max(p["probabilities"]) >= 0.95 and p["classes"] > 0:
            feat.feature = p["classes"]
            features.append(p["classes"])
            feature_snips.append(characters[idx])
        else:
            feat.feature = -1
        data.append(feat)

    print ("Found features")            
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
        if data[i].feature > 0:
            data[i].param.append(orients[feature_count])
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