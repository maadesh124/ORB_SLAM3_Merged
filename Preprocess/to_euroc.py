import cv2
import os

def video_to_euroc_bw_ns(video_path, output_root="output", base_timestamp_ns=1403636579763555584):
    # Enforce directory structure: output_root/mav0/cam0/data
    cam0_dir = os.path.join(output_root, "mav0", "cam0")
    data_dir = os.path.join(cam0_dir, "data")
    os.makedirs(data_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError(f"Cannot open video file: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    dt_ns = int(1e9 / fps)

    timestamps_ns = []
    frame_index = 0
    target_resolution = (1080, 720)  # width x height

    print(f"Extracting grayscale frames at {fps:.2f} FPS...")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Resize to 1080x720 if needed
        if gray.shape[1] != target_resolution[0] or gray.shape[0] != target_resolution[1]:
            gray = cv2.resize(gray, target_resolution, interpolation=cv2.INTER_AREA)

        timestamp_ns = base_timestamp_ns + frame_index * dt_ns
        filename = f"{timestamp_ns}.png"
        filepath = os.path.join(data_dir, filename)

        cv2.imwrite(filepath, gray)
        timestamps_ns.append(timestamp_ns)
        frame_index += 1

    cap.release()
    print(f"✅ Saved {frame_index} grayscale frames to {data_dir}")

    # Save data.csv
    data_csv_path = os.path.join(cam0_dir, "data.csv")
    with open(data_csv_path, 'w') as f:
        f.write("#timestamp,filename\n")
        for ts in timestamps_ns:
            f.write(f"{ts},{ts}.png\n")

    # Save timestamps.txt
    timestamps_txt_path = os.path.join(cam0_dir, "timestamps.txt")
    with open(timestamps_txt_path, 'w') as f:
        for ts in timestamps_ns:
            f.write(f"{ts}\n")

    print(f"✅ data.csv and timestamps.txt saved in {cam0_dir}")

# CLI usage
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Convert video to EuRoC grayscale dataset at 1080x720 resolution with nanosecond timestamps.")
    parser.add_argument("video", help="Path to input .mp4 video")
    parser.add_argument("--output", default="output", help="Output root folder (default: ./output)")
    args = parser.parse_args()

    video_to_euroc_bw_ns(args.video, args.output)

