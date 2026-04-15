import cv2
import os
import glob
import re

def compare_features(query_img, ref_img):
    # Initialize ORB detector
    orb = cv2.ORB_create(nfeatures=1000)
    
    # Find the keypoints and descriptors
    kp1, des1 = orb.detectAndCompute(query_img, None)
    kp2, des2 = orb.detectAndCompute(ref_img, None)
    
    if des1 is None or des2 is None:
        return 0
    
    # Create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    
    # Match descriptors
    matches = bf.match(des1, des2)
    
    # Sort them in the order of their distance
    matches = sorted(matches, key = lambda x:x.distance)
    
    # Good matches (low distance)
    good_matches = [m for m in matches if m.distance < 50]
    
    return len(good_matches)

def find_best_match(query_path, ref_dir):
    query_img = cv2.imread(query_path)
    if query_img is None:
        print(f"Failed to load query image {query_path}")
        return
    
    best_matches = 0
    best_ref = None
    
    ref_files = glob.glob(os.path.join(ref_dir, "*.png"))
    print(f"Comparing {query_path} against {len(ref_files)} reference images...")
    
    for ref_path in ref_files:
        ref_img = cv2.imread(ref_path)
        if ref_img is None:
            continue
            
        num_matches = compare_features(query_img, ref_img)
        
        # Extract pose from filename: frame_000000_x0.00_y0.00.png
        match = re.search(r'_x(-?\d+\.\d+)_y(-?\d+\.\d+)', ref_path)
        pose_str = f"x={match.group(1)}, y={match.group(2)}" if match else "unknown"
        
        print(f"  {os.path.basename(ref_path)} ({pose_str}): {num_matches} matches")
        
        if num_matches > best_matches:
            best_matches = num_matches
            best_ref = (ref_path, pose_str)
            
    if best_ref:
        print(f"\nBest match: {os.path.basename(best_ref[0])}")
        print(f"Estimated reference pose: {best_ref[1]}")
        print(f"Confidence (matches): {best_matches}")
    else:
        print("No good match found.")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("query_image")
    parser.add_argument("reference_dir")
    args = parser.parse_args()
    
    find_best_match(args.query_image, args.reference_dir)
