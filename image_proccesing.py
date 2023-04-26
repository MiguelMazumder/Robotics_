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

    bound_low = (0, 0, 0)
    bound_high = (0, 0, 0)
    return bound_low, bound_high


def image_patch(img, x, y, w, h):
    """ Returns a region of interest of img specified by box """
    # check box against the boundaries of the image
    box = [y, x, y + h, x + w]
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


def image_rectangle(img, x, y, w, h):
    """ Adds a green rectangle to the image """
    # This implementation is a stub. You should implement your own code here.

    return img


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

    # This implementation is a stub. Substitute with your own code.
    x_centroid = 0
    return x_centroid


def pixel_count(img):
    """
    Count how many pixels in the image are non-zero (positive label)
    and zero (negative label)
    """
    # This implementation is a stub. Substitute with your own code.

    nb_positive = np.count_nonzero(img)
    nb_negative = img.size - nb_positive
    return nb_positive, nb_negative


def precision_recall(true_positive, false_positive, false_negative):
    """
    Return precision and recall given counts
    """
    # This implementation is a stub. Substitute with your own code.

    precision = 0
    recall = 0
    return precision, recall


def segmentation_statistics(filename_positive, filename_negative):
    """
    Print segmentation statistics for two files with known segmentation
    """
    # This implementation is a stub. Substitute with your own code.

    precision = 0
    recall = 0
    return precision, recall


def image_centroid_test():
    # load test image
    img = cv2.imread('line-test.jpg')
    # make segmented image
    lb, ub = classifier_parameters()
    img_seg = cv2.inRange(img, lb, ub)
    # compute centroid
    x = image_centroid_horizontal(img_seg)
    # make img color
    color = image_one_to_three_channels(img_seg)
    # add line on color img
    line = image_line_vertical(color, x)
    # show images
    cv2.imshow('test_original', img)
    cv2.waitKey()
    cv2.imshow('test_segmented', line)
    cv2.waitKey()


if __name__ == '__main__':
    test()
