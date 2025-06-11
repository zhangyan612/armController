import time
import numpy as np
from PIL import ImageGrab
import cv2
import os

def get_bottom_bar_screenshot(height=100, right_exclude_width=200):
    screen = ImageGrab.grab()
    screen_np = np.array(screen)
    frame = cv2.cvtColor(screen_np, cv2.COLOR_RGB2BGR)

    screen_height, screen_width, _ = frame.shape
    cropped = frame[screen_height - height : screen_height, 0 : screen_width - right_exclude_width]
    return cropped

def compare_images(img1, img2, threshold=30):
    diff = cv2.absdiff(img1, img2)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    change_count = cv2.countNonZero(thresh)
    return change_count, thresh

def main(save_images=False):
    if save_images:
        os.makedirs("taskbar_captures", exist_ok=True)
    
    print("Monitoring taskbar (excluding clock) every 60s...")

    prev_image = get_bottom_bar_screenshot()

    if save_images:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        cv2.imwrite(f"taskbar_captures/taskbar_{timestamp}.png", prev_image)

    while True:
        time.sleep(60)
        curr_image = get_bottom_bar_screenshot()
        timestamp = time.strftime("%Y%m%d_%H%M%S")

        if save_images:
            filename = f"taskbar_captures/taskbar_{timestamp}.png"
            cv2.imwrite(filename, curr_image)

        change_count, _ = compare_images(prev_image, curr_image)

        if change_count > 0:
            print(f"[{timestamp}] Change detected! Pixel diff count: {change_count}")
        else:
            print(f"[{timestamp}] No change detected.")

        prev_image = curr_image

if __name__ == "__main__":
    main(save_images=False)  # Set to True if you want to keep screenshots
