import cv2
import os
import glob
import re
import numpy as np

def compare_features(query_img, ref_img):
    # Initialize ORB detector
    orb = cv2.ORB_create(nfeatures=2000)
    
    # Find the keypoints and descriptors
    kp1, des1 = orb.detectAndCompute(query_img, None)
    kp2, des2 = orb.detectAndCompute(ref_img, None)
    
    if des1 is None or des2 is None or len(des1) < 10 or len(des2) < 10:
        return 0
    
    # Use FLANN based matcher for ORB (using LSH index)
    FLANN_INDEX_LSH = 6
    index_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2
    search_params = dict(checks=50)
    
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    
    try:
        matches = flann.knnMatch(des1, des2, k=2)
    except cv2.error:
        return 0

    # Store all the good matches as per Lowe's ratio test.
    good = []
    for m_n in matches:
        if len(m_n) == 2:
            m, n = m_n
            if m.distance < 0.7 * n.distance:
                good.append(m)
    
    if len(good) < 10:
        return len(good)

    # Find Homography using RANSAC to count inliers
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    
    if mask is None:
        return len(good)
        
    inliers = np.sum(mask)
    return int(inliers)

def find_best_match(query_path, ref_dir):
    query_img = cv2.imread(query_path)
    if query_img is None:
        print(f"Failed to load query image {query_path}")
        return
    
    best_matches = 0
    best_ref = None
    
    ref_files = glob.glob(os.path.join(ref_dir, "*.png"))
    print(f"Comparing {query_path} against {len(ref_files)} reference images...")
    
    results = []
    for ref_path in ref_files:
        ref_img = cv2.imread(ref_path)
        if ref_img is None:
            continue
            
        num_inliers = compare_features(query_img, ref_img)
        
        # Extract pose from filename: frame_000000_x0.00_y0.00.png
        match = re.search(r'_x(-?\d+\.\d+)_y(-?\d+\.\d+)', ref_path)
        pose_str = f"x={match.group(1)}, y={match.group(2)}" if match else "unknown"
        
        results.append((num_inliers, ref_path, pose_str))
        
    # Sort results by inliers
    results.sort(key=lambda x: x[0], reverse=True)
    
    for num_inliers, ref_path, pose_str in results:
        print(f"  {os.path.basename(ref_path)} ({pose_str}): {num_inliers} inliers")
        if best_ref is None or num_inliers > best_matches:
            best_matches = num_inliers
            best_ref = (ref_path, pose_str)
            
    if best_ref and best_matches >= 10: # Threshold for confidence
        print(f"\nBest match: {os.path.basename(best_ref[0])}")
        print(f"Estimated reference pose: {best_ref[1]}")
        print(f"Confidence (inliers): {best_matches}")
    else:
        print("\nNo confident match found.")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("query_image")
    parser.add_argument("reference_dir")
    args = parser.parse_args()
    
    find_best_match(args.query_image, args.reference_dir)
