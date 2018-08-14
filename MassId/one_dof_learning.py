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
    theta_interested = np.concatenate([theta[1000:2500], theta[4600:5800],
                                       theta[7800:8800], theta[11000:11800],
                                       theta[13500:14500], theta[16000:17500],
                                       theta[22000:28000]])
    trq_interested = np.concatenate([trq[1000:2500], trq[4600:5800],
                                     trq[7800:8800], trq[11000:11800],
                                     trq[13500:14500], trq[16000:17500],
                                     trq[22000:28000]])
    return theta_interested, trq_interested
    # return theta, trq

def learn(args, _theta, _trq):
    theta = tf.placeholder("float")
    trq = tf.placeholder("float")
    mass = tf.Variable(3.8, name="mass")
    # rx = tf.Variable(0.35, name="rx")
    rx = tf.Variable(0.85, name="rx")
    pred = tf.multiply(tf.multiply(tf.multiply(mass, rx), 9.81), tf.cos(theta))
    cost = tf.reduce_sum(tf.pow(pred - trq, 2))
    optimizer = tf.train.GradientDescentOptimizer(args.learning_rate).minimize(cost)
    init = tf.global_variables_initializer()
    tf.summary.scalar("loss", cost)
    merged_summary_op = tf.summary.merge_all()

    with tf.Session() as sess:
        sess.run(init)
        summary_writer = tf.summary.FileWriter(args.log_dir, graph=tf.get_default_graph())
        for epoch in range(args.num_epoch):
            avg_cost = 0.
            num_batch_per_epoch= int(_theta.shape[0]/args.batch_size)
            shuffle_idx = np.random.permutation(np.arange(_theta.shape[0]))
            mixed_theta = _theta[shuffle_idx]
            mixed_trq = _trq[shuffle_idx]
            for i in range(num_batch_per_epoch):
                start_idx = i*args.batch_size
                end_idx = min((i+1)*args.batch_size, _theta.shape[0])
                _, summary, c = sess.run([optimizer, merged_summary_op, cost], feed_dict={theta: mixed_theta[start_idx:end_idx], trq: mixed_trq[start_idx:end_idx]})
                # _, summary, c = sess.run([optimizer, merged_summary_op, cost], feed_dict={theta: _theta[start_idx:end_idx], trq: _trq[start_idx:end_idx]})
                summary_writer.add_summary(summary, epoch * num_batch_per_epoch + i)
                avg_cost += c / num_batch_per_epoch

            print("Epoch:", '%04d' % (epoch+1), "cost=", "{:.9f}".format(avg_cost), \
                  "mass=", sess.run(mass), "rx=", sess.run(rx))

        print("Optimization Finished!")
        training_cost = sess.run(cost, feed_dict={theta: _theta, trq: _trq})
        print("Training cost=", training_cost, "mass=", sess.run(mass), "rx=", sess.run(rx), '\n')
        mass_result = sess.run(mass)
        rx_result = sess.run(rx)

    print("Run the command line:\n" \
          "--> tensorboard --logdir=/tmp/tensorflow_logs " \
          "\nThen open http://0.0.0.0:6006/ into your web browser")

    return mass_result, rx_result

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", type=str, default="data")
    parser.add_argument("--learning_rate", type=float, default=0.0001)
    parser.add_argument("--num_epoch", type=int, default=10)
    parser.add_argument("--batch_size", type=int, default=1)
    parser.add_argument("--log_dir", type=str, default='/tmp/tensorflow_logs/example/')
    args = parser.parse_args()

    theta = readData(args.data_dir+"/massID_theta.txt", 1)
    trq = readData(args.data_dir+"/massID_torque.txt", 1)
    theta, trq = preprocess(theta, trq)
    mass, rx = learn(args, theta, trq)

if __name__ == "__main__":
    main()
