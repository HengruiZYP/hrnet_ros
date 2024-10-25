"""utils"""
import cv2
import numpy as np
import math


def get_color(idx):
    """
    Get a color based on the given index.

    Args:
        idx (int): The index to calculate the color.

    Returns:
        tuple: A tuple representing the RGB values of the color.

    """
    idx = idx * 3
    color = ((37 * idx) % 255, (17 * idx) % 255, (29 * idx) % 255)
    return color


def calculate_angle(point1, point2):   
    """    
    Calculate the angle between two points.
    Args:
        point1 (tuple): (x, y) coordinates of the first point.
        point2 (tuple): (x, y) coordinates of the second point.

    Returns:
        float: The angle in degrees.    
    """    
    delta_x = point2[0] - point1[0]    
    delta_y = point2[1] - point1[1]    
    angle = math.degrees(math.atan2(delta_y, delta_x))    
    return angle


def extract_arm_info(skeletons, visual_thresh=0.6):   
    """    
    Extract the position and angle information of the arms from the skeletons.   
    Args:        
                skeletons (list): List of skeletons, where each skeleton is a numpy array of shape (17, 3).        
            visual_thresh (float): The threshold for visualizing keypoints.    
    Returns:
            list: A list of dictionaries containing arm information for each skeleton.    
    """    
    arm_info_list = []    
    for skeleton in skeletons:
        if skeleton[5, 2] < visual_thresh or skeleton[7, 2] < visual_thresh or skeleton[9, 2] < visual_thresh:
            left_upper_arm_info = None
            left_lower_arm_info = None 
        else:
            left_shoulder = skeleton[5, :2]
            left_elbow = skeleton[7, :2]
            left_wrist = skeleton[9, :2]
            left_upper_arm_angle = calculate_angle(left_shoulder, left_elbow)
            left_lower_arm_angle = calculate_angle(left_elbow, left_wrist)
            left_upper_arm_info = {
                'start': left_shoulder,
                'end': left_elbow, 
                'angle': left_upper_arm_angle
            }
            left_lower_arm_info = {
                'start': left_elbow, 
                'end': left_wrist, 
                'angle': left_lower_arm_angle
            }
        if skeleton[6, 2] < visual_thresh or skeleton[8, 2] < visual_thresh or skeleton[10, 2] < visual_thresh:
            right_upper_arm_info = None
            right_lower_arm_info = None
        else:
            right_shoulder = skeleton[6, :2]
            right_elbow = skeleton[8, :2]
            right_wrist = skeleton[10, :2]
            right_upper_arm_angle = calculate_angle(right_shoulder, right_elbow)
            right_lower_arm_angle = calculate_angle(right_elbow, right_wrist)
            right_upper_arm_info = {
                'start': right_shoulder,
                'end': right_elbow,
                'angle': right_upper_arm_angle
            }            
            right_lower_arm_info = {
                'start': right_elbow,
                'end': right_wrist,
                'angle': right_lower_arm_angle
            }

        arm_info = {
            #'left_upper_arm': left_upper_arm_info,
            'left_lower_arm': left_lower_arm_info,
            #'right_upper_arm': right_upper_arm_info,
            'right_lower_arm': right_lower_arm_info
        }
        arm_info_list.append(arm_info)
    return arm_info_list


def VisKeypointDetection(
    imgfile,
    results,
    visual_thresh=0.6,
    ids=None,
):
    """visualize_pose"""
    if len(results) == 0:
        return imgfile
    skeletons = results["keypoint"]
    scores = results["score"]
    skeletons = np.array(skeletons)
    kpt_nums = 17
    if len(skeletons) > 0:
        kpt_nums = skeletons.shape[1]

    if kpt_nums == 17:  # plot coco keypoint
        EDGES = [
            (0, 1),
            (0, 2),
            (1, 3),
            (2, 4),
            (3, 5),
            (4, 6),
            (5, 7),
            (6, 8),
            (7, 9),
            (8, 10),
            (5, 11),
            (6, 12),
            (11, 13),
            (12, 14),
            (13, 15),
            (14, 16),
            (11, 12),
        ]
    else:  # plot mpii keypoint
        EDGES = [
            (0, 1),
            (1, 2),
            (3, 4),
            (4, 5),
            (2, 6),
            (3, 6),
            (6, 7),
            (7, 8),
            (8, 9),
            (10, 11),
            (11, 12),
            (13, 14),
            (14, 15),
            (8, 12),
            (8, 13),
        ]
    NUM_EDGES = len(EDGES)

    colors = [
        [255, 0, 0],
        [255, 85, 0],
        [255, 170, 0],
        [255, 255, 0],
        [170, 255, 0],
        [85, 255, 0],
        [0, 255, 0],
        [0, 255, 85],
        [0, 255, 170],
        [0, 255, 255],
        [0, 170, 255],
        [0, 85, 255],
        [0, 0, 255],
        [85, 0, 255],
        [170, 0, 255],
        [255, 0, 255],
        [255, 0, 170],
        [255, 0, 85],
    ]

    img = cv2.imread(imgfile) if type(imgfile) == str else imgfile

    color_set = results["colors"] if "colors" in results else None

    if "bbox" in results and ids is None:
        bboxs = results["bbox"]
        for j, rect in enumerate(bboxs):
            xmin, ymin, xmax, ymax = rect
            color = (
                colors[0] if color_set is None else colors[color_set[j] % len(colors)]
            )
            cv2.rectangle(img, (xmin, ymin), (xmax, ymax), color, 1)

    canvas = img.copy()
    for i in range(kpt_nums):
        for j in range(len(skeletons)):
            if skeletons[j][i, 2] < visual_thresh:
                continue
            if ids is None:
                color = (
                    colors[i]
                    if color_set is None
                    else colors[color_set[j] % len(colors)]
                )
            else:
                color = get_color(ids[j])

            cv2.circle(
                canvas,
                tuple(skeletons[j][i, 0:2].astype("int32")),
                2,
                color,
                thickness=-1,
            )

    stickwidth = 2

    for i in range(NUM_EDGES):
        for j in range(len(skeletons)):
            edge = EDGES[i]
            if (
                skeletons[j][edge[0], 2] < visual_thresh
                or skeletons[j][edge[1], 2] < visual_thresh
            ):
                continue

            cur_canvas = canvas.copy()
            X = [skeletons[j][edge[0], 1], skeletons[j][edge[1], 1]]
            Y = [skeletons[j][edge[0], 0], skeletons[j][edge[1], 0]]
            mX = np.mean(X)
            mY = np.mean(Y)
            length = ((X[0] - X[1]) ** 2 + (Y[0] - Y[1]) ** 2) ** 0.5
            angle = math.degrees(math.atan2(X[0] - X[1], Y[0] - Y[1]))
            polygon = cv2.ellipse2Poly(
                (int(mY), int(mX)), (int(length / 2), stickwidth), int(angle), 0, 360, 1
            )
            if ids is None:
                color = (
                    colors[i]
                    if color_set is None
                    else colors[color_set[j] % len(colors)]
                )
            else:
                color = get_color(ids[j])
            cv2.fillConvexPoly(cur_canvas, polygon, color)
            canvas = cv2.addWeighted(canvas, 0.4, cur_canvas, 0.6, 0)
        
    arm_info_list = extract_arm_info(skeletons)

    return canvas, arm_info_list


