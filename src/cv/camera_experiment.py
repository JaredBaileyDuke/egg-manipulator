import cv2

def main():
    # Open the default USB camera (usually index 0)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Unable to open the USB camera")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame")
            break

        # Display the current frame
        cv2.imshow('USB Camera', frame)
        
        # Exit when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Quitting...")
            break

    # Clean up
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()