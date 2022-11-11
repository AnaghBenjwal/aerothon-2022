import cv2

img = cv2.imread('TargetArea.jpg')
imgcopy = img
imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(imgray, 185, 255, cv2.THRESH_BINARY_INV)

# edges = cv2.Canny(imgray, 127, 255)
contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

print('len(contours): ', len(contours))
print('len(contuors[0]: ', len(contours[0]))

centers = []
for contour in contours:
    # compute the center of the contour
    M = cv2.moments(contour)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    centers.append([cx, cy])
    cv2.circle(imgcopy, (cx, cy), 3, (0, 0, 0), -1)
    # approx = cv2.approxPolyDP(contour, epsilon, True)
    # print(approx)

cv2.drawContours(imgcopy, contours, -1, (0, 255, 0), 3)

print('len(centers): ', len(centers))
print('centers: ', centers)
for center in centers:
    print('center: ', center)


centavg_x = 0
centavg_y = 0
for center in centers:
    centavg_x += int((center[0]/len(centers)))
    centavg_y += int((center[1]/len(centers)))

print('average center: ', centavg_x, centavg_y)


cv2.imshow('center', imgcopy)
cv2.waitKey(0)
cv2.destroyAllWindows()