import cv2
import numpy as np
import math

def main():
    # Open the default USB webcam (0 can be replaced with your device index if needed)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return

    # Load the predefined dictionary of ArUco markers (change if necessary)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()

    # Define a known marker length in meters (adjust to your marker's actual size)
    marker_length = 0.05

    print("Starting ArUco marker detection. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame.")
            break
        
        # cv2.imshow('Original', frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect markers in the image
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            # Draw borders for all detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            h, w = frame.shape[:2]
            # Dummy camera calibration parameters (for demonstration)
            camera_matrix = np.array([[800, 0, w/2],
                                      [0, 800, h/2],
                                      [0,   0,   1]], dtype=np.float32)
            dist_coeffs = np.zeros((5, 1))

            for i, corner in enumerate(corners):
                marker_id = ids[i][0]
                pts = corner[0]  # Four corner points

                # Compute center of the marker
                center_x = int(np.mean(pts[:, 0]))
                center_y = int(np.mean(pts[:, 1]))

                # Calculate orientation based on the first edge (corner 0 to corner 1)
                dx = pts[1][0] - pts[0][0]
                dy = pts[1][1] - pts[0][1]
                angle = math.degrees(math.atan2(dy, dx))

                # Compute size based on average side length (in pixels)
                sides = [np.linalg.norm(pts[j] - pts[(j+1) % 4]) for j in range(4)]
                pixel_size = sum(sides) / 4

                # Estimate perspective distortion: compute deviations of inner angles from 90°
                angles = []
                for j in range(4):
                    p0 = pts[j - 1]
                    p1 = pts[j]
                    p2 = pts[(j + 1) % 4]
                    v1 = p0 - p1
                    v2 = p2 - p1
                    # Clamp dot product ratio to avoid domain issues
                    ratio = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                    ratio = max(min(ratio, 1.0), -1.0)
                    ang = math.degrees(math.acos(ratio))
                    angles.append(ang)
                distortion = max(abs(a - 90) for a in angles)

                # Occlusion detection heuristic:
                # If one side is significantly shorter than others, assume partial occlusion.
                occluded = (max(sides) / min(sides)) > 1.5

                # Pose estimation (translation and rotation) provided a known marker size
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, marker_length, camera_matrix, dist_coeffs)
                # Draw coordinate axis for the marker
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_length * 0.5)

                # Extract inner pattern using perspective transform
                pts_dst = np.array([[0, 0], [100, 0], [100, 100], [0, 100]], dtype=np.float32)
                M = cv2.getPerspectiveTransform(pts.astype(np.float32), pts_dst)
                inner_pattern = cv2.warpPerspective(gray, M, (100, 100))
                cv2.imshow(f'Marker {marker_id} Pattern', inner_pattern)

                # Overlay text information on the frame
                info_text = (f"ID: {marker_id}, Angle: {angle:.1f}°, PixelSize: {pixel_size:.1f}, "
                             f"Distort: {distortion:.1f}°, Occluded: {occluded}, "
                             f"tvec: [{tvec[0][0][0]:.2f},{tvec[0][0][1]:.2f},{tvec[0][0][2]:.2f}]")
                cv2.putText(frame, info_text, (center_x - 150, center_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Draw each corner as a small circle
                for pt in pts:
                    cv2.circle(frame, (int(pt[0]), int(pt[1])), 4, (255, 0, 0), -1)

                # Print detailed info to the console
                print(f"\nMarker {marker_id}:")
                print(f"  Corners: {np.int8(pts)}")
                print(f"  Center: ({center_x}, {center_y})")
                print(f"  Orientation: {angle:.1f}°")
                print(f"  Pixel Size (avg side length): {pixel_size:.1f}")
                print(f"  Perspective Distortion (max deviation): {distortion:.1f}°")
                print(f"  Occlusion Detected: {occluded}")
                print(f"  Translation (tvec): {tvec[0][0]}")

        cv2.imshow('ArUco Marker Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()