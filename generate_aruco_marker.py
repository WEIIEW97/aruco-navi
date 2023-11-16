import cv2
import numpy as np
import os

aruco_marker_dict_info = [
    ("DICT_4X4_1000", cv2.aruco.DICT_4X4_1000),
    ("DICT_5X5_1000", cv2.aruco.DICT_5X5_1000),
    ("DICT_6X6_50", cv2.aruco.DICT_6X6_50),
    ("DICT_6X6_100", cv2.aruco.DICT_6X6_100),
    ("DICT_6X6_250", cv2.aruco.DICT_6X6_250),
    ("DICT_6X6_1000", cv2.aruco.DICT_6X6_1000),
    ("DICT_7X7_50", cv2.aruco.DICT_7X7_50),
    ("DICT_7X7_100", cv2.aruco.DICT_7X7_100),
    ("DICT_7X7_250", cv2.aruco.DICT_7X7_250),
    ("DICT_7X7_1000", cv2.aruco.DICT_7X7_1000),
    ("DICT_ARUCO_ORIGINAL", cv2.aruco.DICT_ARUCO_ORIGINAL),
    ("DICT_APRILTAG_16h5", cv2.aruco.DICT_APRILTAG_16h5),
    ("DICT_APRILTAG_25h9", cv2.aruco.DICT_APRILTAG_25h9),
    ("DICT_APRILTAG_36h10", cv2.aruco.DICT_APRILTAG_36h10),
    ("DICT_APRILTAG_36h11", cv2.aruco.DICT_APRILTAG_36h11),
]

TOTAL_MARKERS = 250

aruco_marker_dict = dict(aruco_marker_dict_info)


def generate_aruco_marker_opencv(dict_info, idx, length=200):
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(dict_info),
        id=idx,
        sidePixels=length,
    )
    return marker


def drawer(
    fig,
    text,
    font=cv2.FONT_HERSHEY_SIMPLEX,
    font_scale=0.5,
    font_thickness=1,
    text_color=(50, 50, 50),
    text_margin=20,
):
    coord_x = text_margin
    coord_y = fig.shape[0] - text_margin

    cv2.putText(
        fig, text, (coord_x, coord_y), font, font_scale, text_color, font_thickness
    )


if __name__ == "__main__":
    dict_choose = aruco_marker_dict["DICT_6X6_250"]
    markers = []
    for i in range(TOTAL_MARKERS):
        marker = generate_aruco_marker_opencv(dict_choose, i, 400)
        markers.append(marker)

    outdir = "D:/william/data/aruco_markers"
    outdir_left = os.path.join(outdir, "left")
    outdir_right = os.path.join(outdir, "right")
    outdir_middle = os.path.join(outdir, "middle")

    for d in (outdir_left, outdir_right, outdir_middle):
        os.makedirs(d, exist_ok=True)

    for idx, m in enumerate(markers):
        text_to_put = f"#_{idx}"
        drawer(m, text_to_put, text_color=(30, 30, 30))
        if idx % 10 == 3:
            cv2.imwrite(os.path.join(outdir_left, "{:04d}_aruco.png".format(idx)), m)
        elif idx % 10 == 4:
            cv2.imwrite(os.path.join(outdir_right, "{:04d}_aruco.png".format(idx)), m)
        else:
            cv2.imwrite(os.path.join(outdir_middle, "{:04d}_aruco.png".format(idx)), m)

    print("done!")
