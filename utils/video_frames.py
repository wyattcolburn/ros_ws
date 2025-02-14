import cv2
import os
import glob
from natsort import natsorted
# Function to extract numeric value from filenames
def extract_number(filename):
    """ Extracts the numeric part from the filename to sort correctly. """
    base_name = os.path.basename(filename)  # Get just 'frame_XXX.png'
    num_part = ''.join(filter(str.isdigit, base_name))  # Extract number
    return int(num_part) if num_part else 0  # Convert to int for sorting

# Sort files numerically
frames_dir = "/home/wyattcolburn/ros_ws/utils/ray_frames"
def main():

    # Get all image file paths, adjust extension if needed (e.g., .jpg, .png)
    frame_files = glob.glob(os.path.join(frames_dir, "*.png"))

    frame_files = sorted(frame_files, key=extract_number)
    print(frame_files)
    if not frame_files:
        print("No frames found in the directory.")
        exit()

    frame_delay = 33 # ~33ms for ~30FPS

    for frame_path in frame_files:
        frame = cv2.imread(frame_path)

        if frame is None:
            print(f"Error loading frame: {frame_path}")
            continue

        cv2.imshow("Video Playback", frame)

        # Wait for frame_delay, exit if 'q' is pressed
        if cv2.waitKey(frame_delay) & 0xFF == ord('q'):
            break

# Cleanup
    cv2.destroyAllWindows()


if __name__== "__main__":
    main()
