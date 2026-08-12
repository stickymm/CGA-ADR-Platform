import cv2
import imutils

cap = cv2.VideoCapture(0)

# IMPORTANT: Make sure this matches the grid size of your physical marker!
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

print("Starting feed... Hold a marker up to the camera.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = imutils.resize(frame, width=600)

    # Detect
    corners, ids, rejected = detector.detectMarkers(frame)

    # If ANY marker is detected, draw it and print its ID
    if ids is not None:
        print(f"Detected Marker IDs: {ids.flatten()}")
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        cv2.putText(frame, "FOUND APRIL TAG!", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

    cv2.imshow("ArUco Diagnostic Test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
