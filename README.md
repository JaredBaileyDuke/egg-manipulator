# egg-manipulator

## Project Overview
This project is for a custom robot hand and accompanying remote control. The robot hand is actuated by 6 servos:
- 2 servos are used for the direct drive gripper.
- 4 servos are used for soft robotics.

The remote control wirelessly communicates with the robot gripper via ESP-Now. It uses 12 time-of-flight sensors, allowing the user to control the robot gripper like playing a keyboard.

The project also incorporates computer vision, utilizing ArUco markers placed on the egg. These markers enable the system to determine the orientation of the egg. The ultimate goal is to:
1. Pick up an egg in any orientation.
2. Move and place the egg.
3. Regrasp the egg in the robot hand, if necessary.

## Folder Structure
```
egg-manipulator/
├── assets/
│   ├── aruco_markers_0.pdf
│   ├── aruco_markers_1.pdf
├── rc_gripper/
│   └── generic_code/
│       ├── hand_board_07.ino
│       ├── robot_hand_06.ino
├── src/
│   └── cv/
│       ├── camera_experiment.py
│       ├── cv_experiment.py
```

### Key Components
- **`main.py`**: Entry point for the project.
- **`assets/`**: Contains ArUco marker PDFs (`aruco_markers_0.pdf` and `aruco_markers_1.pdf`) for use on the egg.
- **`rc_gripper/generic_code/`**: Contains Arduino code for the robot hand and remote control:
  - `hand_board_07.ino`: Code for the remote control.
  - `robot_hand_06.ino`: Code for the robot hand.
- **`src/cv/`**: Contains computer vision scripts:
  - `camera_experiment.py`: Basic camera testing script.
  - `cv_experiment.py`: Advanced script for detecting ArUco markers and estimating egg orientation.

## How to Run the Code
1. **Computer Vision**:
   - Navigate to the `src/cv/` directory.
   - Run `cv_experiment.py` to detect ArUco markers and estimate the egg's orientation:
     ```bash
     python src/cv/cv_experiment.py
     ```

2. **Robot Hand and Remote Control**:
   - Flash the Arduino code in `rc_gripper/generic_code/` to the respective devices:
     - `hand_board_07.ino` for the remote control.
     - `robot_hand_06.ino` for the robot hand.

3. **Testing the Camera**:
   - Run `camera_experiment.py` to test the USB camera:
     ```bash
     python src/cv/camera_experiment.py
     ```

## Notes
- The ArUco markers in the `assets/` folder are essential for the computer vision system. Print and attach them to the egg for proper orientation detection.
- Ensure the ESP-Now communication is properly configured between the remote control and the robot hand.

