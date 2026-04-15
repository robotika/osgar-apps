import cv2
import os
import glob
import re
import numpy as np

def compare_features(query_img, ref_img, visualize=False):
    # Initialize ORB detector
    orb = cv2.ORB_create(nfeatures=2000)
    
    # Find the keypoints and descriptors
    kp1, des1 = orb.detectAndCompute(query_img, None)
    kp2, des2 = orb.detectAndCompute(ref_img, None)
    
    if des1 is None or des2 is None or len(des1) < 10 or len(des2) < 10:
        return 0, None
    
    # Use FLANN based matcher for ORB (using LSH index)
    FLANN_INDEX_LSH = 6
    index_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 6,
                   key_size = 12,
                   multi_probe_level = 1)
    search_params = dict(checks=50)
    
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    
    try:
        matches = flann.knnMatch(des1, des2, k=2)
    except cv2.error:
        return 0, None

    # Store all the good matches as per Lowe's ratio test.
    good = []
    for m_n in matches:
        if len(m_n) == 2:
            m, n = m_n
            if m.distance < 0.7 * n.distance:
                good.append(m)
    
    if len(good) < 10:
        return len(good), None

    # Find Homography using RANSAC to count inliers
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    
    if mask is None:
        return len(good), None
        
    inliers = int(np.sum(mask))
    
    vis_img = None
    if visualize:
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = mask.flatten().tolist(), # draw only inliers
                   flags = 2)
        vis_img = cv2.drawMatches(query_img, kp1, ref_img, kp2, good, None, **draw_params)
        
    return inliers, vis_img

def find_best_match(query_path, ref_dir, output_vis=None):
    query_img = cv2.imread(query_path)
    if query_img is None:
        print(f"Failed to load query image {query_path}")
        return
    
    best_matches = 0
    best_ref_path = None
    best_vis = None
    best_pose = None
    
    ref_files = glob.glob(os.path.join(ref_dir, "*.png"))
    print(f"Comparing {query_path} against {len(ref_files)} reference images...")
    
    results = []
    for ref_path in ref_files:
        ref_img = cv2.imread(ref_path)
        if ref_img is None:
            continue
            
        num_inliers, vis = compare_features(query_img, ref_img, visualize=True)
        
        match = re.search(r'_x(-?\d+\.\d+)_y(-?\d+\.\d+)', ref_path)
        pose_str = f"x={match.group(1)}, y={match.group(2)}" if match else "unknown"
        
        results.append((num_inliers, ref_path, pose_str, vis))
        
    results.sort(key=lambda x: x[0], reverse=True)
    
    for num_inliers, ref_path, pose_str, vis in results:
        print(f"  {os.path.basename(ref_path)} ({pose_str}): {num_inliers} inliers")
        if num_inliers > best_matches:
            best_matches = num_inliers
            best_ref_path = ref_path
            best_vis = vis
            best_pose = pose_str
            
    if best_ref_path and best_matches >= 10:
        print(f"\nBest match: {os.path.basename(best_ref_path)}")
        print(f"Estimated reference pose: {best_pose}")
        print(f"Confidence (inliers): {best_matches}")
        
        if output_vis:
            cv2.imwrite(output_vis, best_vis)
            print(f"Saved visualization to {output_vis}")
    else:
        print("\nNo confident match found.")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("query_image")
    parser.add_argument("reference_dir")
    parser.add_argument("--out-vis", help="Path to save the best match visualization")
    args = parser.parse_args()
    
    find_best_match(args.query_image, args.reference_dir, args.out_vis)
