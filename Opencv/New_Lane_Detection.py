import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

path = r'C:\Users\Musab Ahmed Pathan\Downloads\challenge_video.mp4'

cap = cv.VideoCapture(path)

x_left = []
x_right = []
y_left = []
y_right = []
co_or_left = []
co_or_right = []
count = 9

while(cap.isOpened()):
    ret, frame = cap.read()
    y_height = frame.shape[0]

    pts1 = np.float32([[520, 520], [850, 520], [330, y_height-50], [1050, y_height-50]])
    pts2 = np.float32([[0, 0], [800, 0], [0, 800], [800, 800]])

    M = cv.getPerspectiveTransform(pts1, pts2)
    Minv = cv.getPerspectiveTransform(pts2, pts1)
    dst = cv.warpPerspective(frame, M, (800, 700))
    dst2 = cv.warpPerspective(dst, Minv, (frame.shape[0]+900, frame.shape[1]))

    dstcopy = dst[:, :, 2]

    clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    cl1 = clahe.apply(dstcopy)

    blur = cv.GaussianBlur(cl1, (5, 5), 1)
    channel = cl1[:, :]
    thresh = (200, 250)
    output = np.zeros_like(channel)
    output[(channel >= thresh[0]) & (channel <= thresh[1])] = 255

    mask = np.zeros_like(output)
    mask[:, 0:300] = 255
    mask[:, 600:] = 255
    mask = cv.bitwise_and(mask, output)

    final_mask = np.zeros_like(mask)
    final_mask = cv.cvtColor(final_mask, cv.COLOR_GRAY2BGR)

    window_size = mask.shape[0]

    histogram = np.sum(mask[mask.shape[0]//2:, :], axis = 0)

    midpoint = np.int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    left_mid = leftx_base
    right_mid = rightx_base

    window_left = mask[:, 0:left_mid+50]
    window_right = mask[:, right_mid-50:]

    pixels_left = np.argwhere(window_left == 255)
    pixels_right = np.argwhere(window_right == 255)
    pixels_right[:, 1] = pixels_right[:, 1] + right_mid-50

    pixels_left[:, [0, 1]] = pixels_left[:, [1, 0]]
    pixels_right[:, [0, 1]] = pixels_right[:, [1, 0]]

    count = count + 1

    if count%5==0:
        for obj in pixels_left:
            x_left.append(obj[0])
            y_left.append(obj[1])

        for obj in pixels_right:
            x_right.append(obj[0])
            y_right.append(obj[1])

    left_fit = np.polyfit(x_left[len(x_left)//10:len(x_left) - 1000], y_left[len(x_left)//10:len(x_left) - 1000], 1)
    right_fit = np.polyfit(x_right[len(x_right)//10:len(x_right) - 1000], y_right[len(x_right)//10:len(x_right) - 1000], 1)

    h, w = mask.shape[:2]

    x = np.arange(w)

    pol1 = lambda x: (x*left_fit[0] + left_fit[1])
    pol2 = lambda x: (x*right_fit[0] + right_fit[1])
    y1 = pol1(x)
    y2 = pol2(x)

    points1 = np.array([[[xi, yi]] for xi, yi in zip(x, y1) if (0 <= xi < w and 0 <= yi < h)]).astype(np.int32)
    points2 = np.array([[[xi, yi]] for xi, yi in zip(x, y2) if (0 <= xi < w and 0 <= yi < h)]).astype(np.int32)

    points = np.concatenate((points1, np.flip(points2,1)))

    cv.fillPoly(final_mask, [points], color=[0, 255, 0])

    dst2 = cv.warpPerspective(final_mask, Minv, (frame.shape[1], frame.shape[0]))

    final = cv.addWeighted(frame, 1, dst2, 0.4, 1)

    cv.imshow('masked', final)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()