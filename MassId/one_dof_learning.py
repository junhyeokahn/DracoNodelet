import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

def readData(fileName_, dim_):
    if not os.path.isfile(fileName_):
        print("File path {} does not exist. Exiting...".format(fileName_))
        sys.exit()
    if dim_ == 1:
        ret = np.ndarray(1)
        tmp = np.ndarray(1)
        with open(fileName_) as fp:
            for line in fp:
                vecList = line.split("\t")
                vecSize = len(vecList)
                for i in range(vecSize-1):
                    tmp[0] = float(vecList[i])
                    if i == 0:
                        ret = np.copy(tmp)
                    else:
                        ret = np.concatenate((ret, tmp), axis=0)
    else:
        ret = np.ndarray(shape=(1,dim_), dtype='float')
        tmp = np.ndarray(shape=(1,dim_), dtype='float');
        isFirst = True
        with open(fileName_) as fp:
            for line in fp:
                for i in range(dim_):
                    tmp[0][i] = float(line.split("\t")[i])
                if isFirst:
                    ret[0] = np.copy(tmp);
                    isFirst = False
                else:
                    ret = np.concatenate((ret, tmp), axis=0)
    return ret

def preprocess(theta, trq):
    # range we want
    # non static movement
    return theta, trq

def learn(args, _theta, _trq):
    theta = tf.placeholder("float")
    trq = tf.placeholder("float")
    mass = tf.Variable(3.8, name="mass")
    rx = tf.Variable(0.35, name="rx")
    pred = tf.multiply(tf.multiply(tf.multiply(mass, rx), 9.81), tf.cos(theta))
    cost = tf.reduce_sum(tf.pow(pred - trq, 2))
    optimizer = tf.train.GradientDescentOptimizer(args.learning_rate).minimize(cost)
    init = tf.global_variables_initializer()

    with tf.Session() as sess:
        sess.run(init)
        for epoch in range(args.num_epoch):
            __import__('ipdb').set_trace()
            sess.run(optimizer, feed_dict={theta: _theta, trq: _trq})

            if (epoch+1) % 50 == 0:
                c = sess.run(cost, feed_dict={theta: _theta, trq: _trq})
                print("Epoch:", '%04d' % (epoch+1), "cost=", "{:.9f}".format(c), \
                    "mass=", sess.run(mass), "rx=", sess.run(rx))

        print("Optimization Finished!")
        training_cost = sess.run(cost, feed_dict={theta: _theta, trq: _trq})
        print("Training cost=", training_cost, "mass=", sess.run(mass), "b=", sess.run(rx), '\n')
        mass_result = sess.run(mass)
        rx_result = sess.run(rx)

    return mass_result, rx_result

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", type=str, default="data")
    parser.add_argument("--learning_rate", type=float, default=0.01)
    parser.add_argument("--num_epoch", type=int, default=1000)
    args = parser.parse_args()

    theta = readData(args.data_dir+"/massID_theta.txt", 1)
    trq = readData(args.data_dir+"/massID_torque.txt", 1)
    theta, trq = preprocess(theta, trq)
    mass, rx = learn(args, theta, trq)

if __name__ == "__main__":
    main()
