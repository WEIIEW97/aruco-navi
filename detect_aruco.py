import cv2
import numpy as np
from generate_aruco_marker import aruco_marker_dict


def detect_markers(image, aruco_dict_info, make_pair=True):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _aruco_info = cv2.aruco.getPredefinedDictionary(aruco_dict_info)
    parameters = cv2.aruco.DetectorParameters()
    # for new apis
    detector = cv2.aruco.ArucoDetector(_aruco_info, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)
    if ids is None:
        return None
    else:
        if not make_pair:
            return corners, ids, rejected_img_points
        else:
            return list(zip(ids, corners))


def find_min_id_corners(pairs):
    min_id = np.Inf
    cnt = 0
    rec = 0
    for pair in pairs:
        if pair[0] < min_id:
            min_id = pair[0]
            rec = cnt
        cnt += 1
    return pairs[rec][1]


def R2ypr(rot):
    n = rot[:, 0]
    o = rot[:, 1]
    a = rot[:, 2]
    ypr = np.zeros(3)
    y = np.arctan2(n[1], n[0])
    p = np.arctan2(-n[2], n[0] * np.cos(y) + n[1] * np.sin(y))
    r = np.arctan2(
        a[0] * np.sin(y) - a[1] * np.cos(y), -o[0] * np.sin(y) + o[1] * np.cos(y)
    )
    ypr[0] = y
    ypr[1] = p
    ypr[2] = r
    return ypr / np.pi * 180.0


def build_intrinsic(intrinsic_param):
    fx = intrinsic_param["fx"]
    fy = intrinsic_param["fy"]
    cx = intrinsic_param["cx"]
    cy = intrinsic_param["cy"]

    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])


def estimate_single_markers(corners, marker_len, cam_matrix, dist_coeffs):
    marker_points = np.array(
        [
            [-marker_len / 2, marker_len / 2, 0],
            [marker_len / 2, marker_len / 2, 0],
            [marker_len / 2, -marker_len / 2, 0],
            [-marker_len / 2, -marker_len / 2, 0],
        ],
        dtype=np.float32,
    )

    _, rvec, tvec = cv2.solvePnP(marker_points, corners, cam_matrix, dist_coeffs)
    return rvec, tvec


def get_ypr_and_translation(
    corner, marker_len, intrinsic_matrix, dist_coef=np.zeros(5)
):
    # opencv 4.8.0 has deleted `cv2.aruco.estimatePoseSingleMarkers`
    # rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
    #     corners=corner,
    #     markerLength=marker_len,
    #     cameraMatrix=intrinsic_matrix,
    #     distCoeffs=dist_coef,
    # )
    rvec, tvec = estimate_single_markers(
        corners=corner,
        marker_len=marker_len,
        cam_matrix=intrinsic_matrix,
        dist_coeffs=dist_coef,
    )
    # convert rvec to rotation matrix
    rotvec, _ = cv2.Rodrigues(rvec)
    # inverse the world2cam to cam2world
    rot_cam2world = np.transpose(rotvec)
    trans_cam2world = -rot_cam2world @ tvec
    ypr = R2ypr(rot_cam2world)
    return ypr, trans_cam2world


def process(image, dict_choose, marker_len, camera_matrix, dist_coef):
    pairs = detect_markers(image, dict_choose)
    corners = find_min_id_corners(pairs)
    ypr, trans = get_ypr_and_translation(corners, marker_len, camera_matrix, dist_coef)
    return ypr, trans


if __name__ == "__main__":
    left_depature_path = "aruco_capture/left_departure.png"
    right_depature_path = "aruco_capture/right_departure.png"

    dict_choose = aruco_marker_dict["DICT_6X6_250"]

    left_image = cv2.imread(left_depature_path)
    right_image = cv2.imread(right_depature_path)

    fx = 610.117
    fy = 608.71
    cx = 316.156
    cy = 249.345
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist_coef = np.zeros(5)

    print("================ left ================")
    pairs = detect_markers(left_image, dict_choose)
    corner = find_min_id_corners(pairs)
    ypr, trans = get_ypr_and_translation(corner, 0.176, camera_matrix, dist_coef)
    print(ypr)
    print(trans)

    print("================ right ================")
    pairs = detect_markers(right_image, dict_choose)
    corner = find_min_id_corners(pairs)
    ypr, trans = get_ypr_and_translation(corner, 0.176, camera_matrix, dist_coef)
    print(ypr)
    print(trans)
    
    rootdir = "aruco_capture"
    dict_choose = aruco_marker_dict["DICT_6X6_250"]
    accept_names = []
    for i in range(100, 201):
        name = "frame_{:04d}.png".format(i)
        pairs = detect_markers(cv2.imread(f"{rootdir}/{name}"), dict_choose)
        if pairs is not None:
            accept_names.append(name)

    print(accept_names)