import os
import subprocess

def get_latest_image_dir(base_path):
    # List all directories in the base path
    dirs = [d for d in os.listdir(base_path) if os.path.isdir(os.path.join(base_path, d))]
    # Get the latest directory by timestamp
    latest_dir = max(dirs, key=lambda d: os.path.getmtime(os.path.join(base_path, d)))
    return os.path.join(base_path, latest_dir)

def create_video_from_images(image_dir, output_video):
    # Define the ffmpeg command
    ffmpeg_cmd = [
        'ffmpeg',
        '-framerate', '5',  # frames per second
        '-pattern_type', 'glob',
        '-i', os.path.join(image_dir, '*.jpg'),
        '-c:v', 'libx264',
        '-pix_fmt', 'yuv420p',
        output_video
        ]
    
    # Run the ffmpeg command
    subprocess.run(ffmpeg_cmd, check=True)

if __name__ == "__main__":
    base_path = 'src/Thesis_Workspace/hds_and_website/ras_reliability_backend/pyflask'  # Change this to your actual base directory
    output_video = 'yolo_video.mp4'          # Change this to your desired output video name
    
    latest_image_dir = get_latest_image_dir(base_path)
    create_video_from_images(latest_image_dir, output_video)
