import cv2


frame = cv2.imread('Screenshot from 2024-09-10 16-53-35.png')

print('már meg kellett volna jelennie')

cv2.imshow("Nézd meg hogy megy", frame)
cv2.waitKey(0)