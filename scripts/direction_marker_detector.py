import math

import cv2
import numpy as np

import vision_common


def hull_score(hull):
    """
    Give a score to a convex hull based on how likely it is to be a direction marker.
    :param hull: convex hull to test
    :return: Score based on the ratio of side lengths and a minimum area
    """
    rect = cv2.minAreaRect(hull)
    shorter_side = min(rect[1])
    longer_side = max(rect[1])

    ratio_score = 0.0

    # check to make sure the size is defined to prevent possible division by 0 error
    if shorter_side != 0 and longer_side != 0:
        # the marker is 6 inches by 4 feet so the ratio of long : short = 8
        ratio_score = 100 - abs((longer_side / shorter_side) - 8)

    # cut off minimum area at 100 px^2
    if cv2.contourArea(hull) < 100:
        return 0

    return ratio_score


def hull_filter(hull):
    """
    Threshold for removing hulls from positive detection.
    :param hull: convex hull to test
    :return: True if it is a good match, False if it was a bad match (false positive).
    """
    score = hull_score(hull)
    if score < 1 or math.isnan(score):
        return False
    return True


def find(img, hue_min=125, hue_max=175, sat_min=240, sat_max=255, val_min=215, val_max=255):
    """
    Detect direction markers. These are the orange markers on the bottom of the pool that point ot the next objective.
    :param img: HSV image from the bottom camera
    :return: a list of tuples indicating the rectangles detected (center, width x height, angle)
    """

    img = np.copy(img)

    bin = vision_common.hsv_threshold(img, hue_min, hue_max, sat_min, sat_max, val_min, val_max)
    cv2.imshow('bin', bin)

    canny = vision_common.canny(bin, 50)

    # find contours after first processing it with Canny edge detection
    contours, hierarchy = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    hulls = vision_common.convex_hulls(contours)
    cv2.drawContours(bin, hulls, -1, 255)

    # remove convex hulls that don't pass our scoring threshold
    hulls = filter(hull_filter, hulls)

    # draw hulls in Blaze Orange
    cv2.drawContours(img, hulls, -1, (0, 102, 255), -1)
    # draw green outlines so we know it actually detected it
    cv2.drawContours(img, hulls, -1, (0, 255, 0), 2)

    rects = map(lambda hull: cv2.minAreaRect(hull), hulls)
    # shape[0] is the number of rows because matrices are dumb
    rects = map(
        lambda rect: ((rect[0][1] / img.shape[1], rect[0][0] / img.shape[0]), rect[1], vision_common.angle(rect)),
        rects)
    # convert to the targeting system of [-1, 1]
    rects = map(lambda rect: (((rect[0][0] * 2) - 1, (rect[0][1] * 2) - 1), rect[1], rect[2]), rects)

    # cv2.imshow('result', img)

    return rects


def nothing(x):
    pass


if __name__ == '__main__':
    # cv2.namedWindow('bin')
    # cv2.createTrackbar('min', 'bin', 0, 255, nothing)
    # cv2.createTrackbar('max', 'bin', 0, 255, nothing)

    img = cv2.imread('sample.jpg', cv2.IMREAD_COLOR)

    # while True:
    #     find(img, val_min=cv2.getTrackbarPos('min', 'bin'), val_max=cv2.getTrackbarPos('max', 'bin'))
    #     print(cv2.getTrackbarPos('min', 'bin'), cv2.getTrackbarPos('max', 'bin'))
    #     cv2.waitKey(1)

    rects = find(img)

    # cv2.imshow('img', img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()