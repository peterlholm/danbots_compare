import cv2
import sys


img = cv2.imread(f"{sys.argv[1]}/image0.png") #cv2.imread("/home/alexander/Documents/Github/SW/calibrationPlatform/client/images_calibration4x6_1-8mmCM4-6/render21/image0.png")
cv2.imshow("orig",img)
cv2.imshow("blue",img[:, :, 0])
cv2.imshow("green",img[:, :, 1])
cv2.imshow("red",img[:, :, 2])
cv2.waitKey(0)


#cv2.imwrite("checkerboardCorrected.png", img)
