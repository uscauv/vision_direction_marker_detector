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

    # the marker is 6 inches by 4 feet so the ratio of long : short = 8
    ratio_score = 1 / abs((longer_side / shorter_side) - 8)

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

class DirectionMarkerDetector:
    def find(self, img):
        """
        Detect direction markers. These are the orange markers on the bottom of the pool that point ot the next objective.
        :param img: HSV image from the bottom camera
        :return: a list of tuples indicating the rectangles detected (center, width x height, angle)
        """

        img = np.copy(img)

        # TODO: get rid of these magic numbers
        bin = vision_common.hsv_threshold(img, 20, 175, 0, 255, 0, 255)

        contours, hierarchy = cv2.findContours(bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hulls = vision_common.convex_hulls(contours)
        cv2.drawContours(bin, hulls, -1, 255)

        # remove convex hulls that don't pass our scoring threshold
        hulls = filter(hull_filter, hulls)

        # draw hulls in Blaze Orange
        cv2.drawContours(img, hulls, -1, (0, 102, 255), -1)
        # draw green outlines so we know it actually detected it
        cv2.drawContours(img, hulls, -1, (0, 255, 0), 2)

        return map(vision_common.angle, map(lambda hull: cv2.minAreaRect(hull), hulls))


img = cv2.imread('sample.jpg', cv2.IMREAD_COLOR)
detector = DirectionMarkerDetector()
rects = detector.find(img)

cv2.waitKey(0)
cv2.destroyAllWindows()