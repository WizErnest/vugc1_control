from cv_bridge import CvBridge, CvBridgeError
from keras.layers import Conv2D, Cropping2D, Dense, Dropout, Flatten, Lambda, MaxPooling2D
from keras.models import Sequential
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
import argparse
import csv
import cv2
import gc
import numpy as np
import rosbag


def get_data(path, np_format=False):
    images = []
    labels = []

    if np_format: # np.savez format, with data labeled as 'images', 'labels', respectively
        dat = np.load(path)
        print('[#get_data]: found files: ', dat.files)
        return dat['images'], dat['labels'] #images, labels

    bridge = CvBridge()
    bag = rosbag.Bag(path)
    topics = [
        '/zed/rgb/image_rect_color',
        '/vugc1_control_drive_parameters'
    ]

    angle = 0.0
    count = 0

    for topic, message, timestamp in bag.read_messages(topics=topics):
        print('{}: [{}]: {}'.format(timestamp, topic, ''))

        if topic == '/zed/rgb/image_rect_color':
            count += 1

            try:
                image = bridge.imgmsg_to_cv2(message)

                # crop and resize
                image = image[188:, 0:672, 0:3]
                image = cv2.resize(image, (320, 160))

		# filter blue values
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_red = np.array([110,50,50])
    		upper_red = np.array([130,255,255])
                mask = cv2.inRange(hsv, lower_red, upper_red)
		image = cv2.bitwise_and(image,image, mask= mask)
		# convert to grayscale
                # weights = [1, 0, 0] # BGR (standard luminescence: [0.114, 0.587, 0.299])
                # weights = np.array([weights]).reshape((1,3))
                # image = cv2.transform(image, weights)

                # cv2.imshow('image', image)
                # cv2.waitKey(0)

                print('[{}] message=({}, {}), shape=({})'.format(count, message.height, message.width, image.shape))

                images.append(image)
                labels.append(angle)
            except CvBridgeError as e:
                print(e)

        elif topic == '/vugc1_control_drive_parameters': # /vugc1_control_drive_parameters
            print('angle={}'.format(angle))
            angle = message.angle

    bag.close()
    del bag
    print('[#get_data]: cleaning up bag: {}'.format(gc.collect()))

    return np.array(images), np.array(labels)


def get_model(activation_type='elu', dropout=0.3):
    '''
    model = Sequential()
    model.add(Cropping2D(cropping=((60, 25), (0, 0)), input_shape=(160, 320, 3)))
    model.add(Lambda(lambda x: (x / 255) - .5))
    model.add(Conv2D(24, (5, 5), strides=(2, 2), activation=activation_type))
    # model.add(MaxPooling2D())
    model.add(Dropout(dropout))
    model.add(Conv2D(36, (5, 5), strides=(2, 2), activation=activation_type))
    # model.add(MaxPooling2D())
    model.add(Dropout(dropout))
    model.add(Conv2D(48, (5, 5), strides=(2, 2), activation=activation_type))
    # model.add(MaxPooling2D())
    model.add(Dropout(dropout))
    model.add(Conv2D(64, (3, 3), activation=activation_type))
    # model.add(MaxPooling2D())
    model.add(Dropout(dropout))
    model.add(Conv2D(64, (3, 3), activation=activation_type))
    # model.add(MaxPooling2D())
    model.add(Dropout(dropout))
    model.add(Flatten())
    model.add(Dense(1162, activation=activation_type))
    model.add(Dense(100, activation=activation_type))
    model.add(Dense(50, activation=activation_type))
    model.add(Dense(10, activation=activation_type))
    model.add(Dense(1))
    '''
    model = Sequential()
    model.add(Conv2D(36, (5, 5), strides=(2, 2), activation=activation_type, input_shape=(160, 320, 3)))
    model.add(Conv2D(64, (3, 3), activation=activation_type))
    model.add(Flatten())
    model.add(Dense(15, activation=activation_type))
    model.add(Dense(10, activation=activation_type))
    model.add(Dense(1))
    return model

def main():
    parser = argparse.ArgumentParser(description='[behavioral cloning model] choose bag')
    parser.add_argument('--bag', type=str, help='path to bag')
    parser.add_argument('--numpy_data', type=str, help='path to numpy saved array')
    args = parser.parse_args()
    if args.numpy_data:
        data_path = args.numpy_data
        data, labels = get_data(data_path, np_format=True)
    else:
        data_path = args.bag
        data, labels = get_data(data_path)
    model = get_model()


    print('[#main]: training the model')
    model.compile(optimizer='adam', loss='mse')
    model.fit(data, labels, batch_size=16, epochs=14, validation_split=.2)

    tokens = (args.bag or args.numpy_data).split('/')
    bag_name = tokens[-1]
    print('[#main]: saving to {}.h5'.format(bag_name))
    model.save('{}.h5'.format(bag_name))

    del model, data, labels
    collected = gc.collect()
    print('[#main]: cleaning up: {}'.format(collected))


if __name__ == '__main__':
    main()
