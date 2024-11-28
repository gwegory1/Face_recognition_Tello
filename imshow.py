import cv2


frame = cv2.imread('Screenshot from 2024-09-10 16-53-35.png')

print('show')

cv2.imshow("Frame", frame)
cv2.waitKey(0)
