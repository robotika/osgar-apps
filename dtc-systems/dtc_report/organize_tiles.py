import os
import shutil

# --- Configuration ---
source_folder = 'downloaded_tiles'
destination_folder = 'tiles'
# ---------------------

if not os.path.exists(source_folder):
    print(f"Error: Source folder '{source_folder}' not found.")
    exit()

print(f"Starting tile organization from '{source_folder}'...")

# Loop through all files in the source folder
for filename in os.listdir(source_folder):
    # Skip directories or files with extensions
    if os.path.isdir(os.path.join(source_folder, filename)) or '.' in filename:
        continue

    if '(' in filename:  # some strange PNG bits
        continue

    # Split the name by the hyphen, e.g., "18-70083-106019" -> ["18", "70083", "106019"]
    parts = filename.split('-')

    if len(parts) == 3:
        z, x, y = parts
        
        # Create the target directory path, e.g., "tiles/18/70083"
        target_dir = os.path.join(destination_folder, z, x)
        os.makedirs(target_dir, exist_ok=True)
        
        # Define the source and destination paths for the file
        source_path = os.path.join(source_folder, filename)
        # Add the .jpeg extension to the destination filename
        destination_path = os.path.join(target_dir, y + '.jpeg')
        
        # Move and rename the file
        shutil.copy(source_path, destination_path)
        print(f"Copy {filename} -> {destination_path}")

print("Tile organization complete.")
