#!/usr/bin/env python3
"""
This is a library of functions for performing color-based image segmentation
of an image and finding its centroid.
"""

import cv2
import numpy as np


def classifier_parameters():
    """
    Returns two sets of thresholds indicating the three lower thresholds
    and upper thresholds for detecting pixels belonging to the track with
    respect to the background.
    """
    # This implementation is a stub. Modify with your thresholds.

    bound_low = (25, 44, 150)
    bound_high = (90, 134, 193)
    return bound_low, bound_high


def image_patch(img, x__, y__, w__, h__):
    """ Returns a region of interest of img specified by box """
    # check box against the boundaries of the image
    box = [y__, x__, y__ + h__, x__ + w__]
    if box[0] < 0:
        box[0] = 0
    if box[1] < 0:
        box[1] = 0
    if box[2] > img.shape[0]:
        box[2] = img.shape[0]
    if box[3] > img.shape[1]:
        box[3] = img.shape[1]

    return img[box[0]:box[2], box[1]:box[3], :]


def image_line_vertical(img, x):
    """ Adds a green 3px vertical line to the image """
    cv2.line(img, (x, 0), (x, img.shape[1]), (0, 255, 0), 3)
    return img


#def image_rectangle(img, x__, y__, w__, h__):
    #""" Adds a green rectangle to the image """
    # This implementation is a stub. You should implement your own code here.

    #return img


def image_one_to_three_channels(img):
    """ Transforms an image from two channels to three channels """
    # First promote array to three dimensions,
    # then repeat along third dimension
    img_three = np.tile(img.reshape(img.shape[0], img.shape[1], 1), (1, 1, 3))
    return img_three


def image_centroid_horizontal(img):
    """
    Compute the horizontal centroid of white pixels in
    a grascale image
    """
    # Assumes that img contains only black and white pixels
    white_pixels = np.argwhere(img == 255)  # Find the indices of all white pixels
    x_coords = white_pixels[:, 1]           # Get the x coordinates of all white pixels

    if len(x_coords) == 0:   # If there are no white pixels, return zero
        return 0

    x_centroid = int(np.median(x_coords))
    return x_centroid


def pixel_count(img):
    """
    Count how many pixels in the image are non-zero (positive label)
    and zero (negative label)
    """
    # This implementation is a stub. Substitute with your own code.
    height, width = img.shape
    # Initialize a counter for non-zero pixels
    count = 0
    # Loop through each pixel in the image
    for y_iter in range(height):
        for x_iter in range(width):
            # Check if the pixel value is non-zero
            if img[y_iter, x_iter] != 0:
                # Increment the counter
                count += 1
    nb_positive=count
    nb_negative = img.size - nb_positive
    return nb_positive, nb_negative


def precision_recall(true_positive, false_positive, false_negative):
    """
    Return precision and recall given counts
    """
    # This implementation is a stub. Substitute with your own code.

    precision = float(true_positive/(true_positive+false_positive))
    recall = float(true_positive/(true_positive+false_negative))
    return precision, recall


def segmentation_statistics(filename_positive, filename_negative):
    """
    Print segmentation statistics for two files with known segmentation
    """
    # This implementation is a stub. Substitute with your own code.
    high_,low_=classifier_parameters()
    img_positive = cv2.imread(filename_positive)
    img_negative = cv2.imread(filename_negative)
    img_positive_hsv=cv2.cvtColor(img_positive,cv2.COLOR_BGR2HSV)
    img_negative_hsv=cv2.cvtColor(img_negative,cv2.COLOR_BGR2HSV)
    #find if it fits in range 0 if it doesnt, 255 if it does
    img_pos_yhat=cv2.inRange(img_positive_hsv,low_,high_)
    img_neg_yhat=cv2.inRange(img_negative_hsv, low_, high_)
    p_1,f_1=pixel_count(img_pos_yhat)
    p_2,f_2=pixel_count(img_neg_yhat)
    precision,recall=precision_recall(p_1, f_2, f_1)
    # Display statistics
    #number of points in image
    print(f"Total Number of points in {filename_positive}: {p_1+f_1}")
    print(f"Total Number of points in {filename_negative}: {p_2+f_2}")
    #number of positives and negatives in each image
    print(f"Total number of positives in {filename_positive}: {p_1}")
    print(f"Total number of negatives in {filename_positive}: {f_1}")
    print(f"Total number of positives in {filename_negative}: {p_2}")
    print(f"Total number of negatives in {filename_negative}: {f_2}")
    #number of false positives and false negatives
    print(f"Number of false positives in {filename_positive}: {0}")
    print(f"Number of false negatives in {filename_positive}: {f_1}")
    print(f"Number of false positives in {filename_negative}: {p_2}")
    print(f"Number of false negatives in {filename_negative}: {0}")
    #print recall
    print(f"Precision: {precision}")
    print(f"Recall: {recall}")
    return precision,recall


def image_centroid_test():
    """random docstring till i figure it out"""
    # load test image
    img = cv2.imread('line-test.jpg')
    # make segmented image
    lb__, ub__ = classifier_parameters()
    img_seg = cv2.inRange(img, lb__, ub__)
    # compute centroid
    x__ = image_centroid_horizontal(img_seg)
    # make img color
    color = image_one_to_three_channels(img_seg)
    # add line on color img
    line = image_line_vertical(color, x__)
    # show images
    cv2.imshow('test_original', img)
    cv2.waitKey()
    cv2.imshow('test_segmented', line)
    cv2.waitKey()
def image_statistics_test():
    """Calls the segmentation statistics as well as write for line and cross"""
    #do it with train and cross
    _,_=segmentation_statistics('train_positive.png', 'train_negative.png')
    _,_=segmentation_statistics('train_positive.png', 'train_negative.png')
    #apply same thresholds to line-training and line-cross
    line_train=cv2.imread('line-training.png')
    train_hsv=cv2.cvtColor(line_train,cv2.COLOR_BGR2HSV)
    line_cross=cv2.imread('line-cross.png')
    cross_hsv=cv2.cvtColor(line_cross,cv2.COLOR_BGR2HSV)
    high_,low_=classifier_parameters()
    train_mask=cv2.inRange(train_hsv,low_,high_)
    cross_mask=cv2.inRange(cross_hsv,low_,high_)
    cv2.imwrite('line-training_mask',train_mask)
    cv2.imwrite('cross-training_mask',cross_mask)

if __name__ == '__main__':
    test()
