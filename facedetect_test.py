from utils import *
import cv2
# Load an image from file
image_name = 'test7.jpeg'
image_path = 'test_images/' + image_name
image = cv2.imread(image_path)
image = cv2.resize(image, (360, 240))

image2 = cv2.imread(image_path)
image2 = cv2.resize(image, (360, 240))

# Check if the image was successfully loaded
if image is None:
    print("Error: Could not load image.")
else:
    print("Image loaded successfully.")

# Detect faces in the image
face, info = findFace(image)

# Detect faces in the image using MediaPipe
face_mp, info = findFaceMediaPipe(image2)

# Display the image with the detected face
if face is not None:
    cv2.imshow("Face Detection", face)
    cv2.imshow("Face Detection MediaPipe", face_mp)
    # Save the images with the detected faces
    cv2.imwrite('result_images/' + image_name + 'face_detection.jpg', face)
    cv2.imwrite('result_images/' + image_name + 'face_detection_mediapipe.jpg', face_mp)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


