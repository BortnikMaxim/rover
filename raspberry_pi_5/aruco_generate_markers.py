import argparse
import cv2
import numpy as np

aruco_dict_map = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def generate_marker(dict_name, marker_id, marker_size, border_bits, out_file, show):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco not found. Install opencv-contrib-python.")

    if dict_name not in aruco_dict_map:
        raise ValueError(f"unknown dictionary: {dict_name}")

    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_map[dict_name])

    # marker image placeholder
    marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8)

    if hasattr(cv2.aruco, "generateImageMarker"):
        # official docs name
        cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size, marker_img, border_bits)
    else:
        cv2.aruco.drawMarker(dictionary, marker_id, marker_size, marker_img, border_bits)

    cv2.imwrite(out_file, marker_img)
    print(f"[ok] saved: {out_file}")

    if show:
        cv2.imshow("marker", marker_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default="marker.png", help="output file (png)")
    ap.add_argument("-d", "--dict", default="DICT_6X6_250", help="dictionary name")
    ap.add_argument("-id", "--marker_id", type=int, default=23, help="marker id")
    ap.add_argument("-s", "--size", type=int, default=200, help="marker size in pixels")
    ap.add_argument("-bb", "--border", type=int, default=1, help="border bits (default 1)")
    ap.add_argument("--show", action="store_true", help="show marker in window")
    args = ap.parse_args()

    generate_marker(
        dict_name=args.dict,
        marker_id=args.marker_id,
        marker_size=args.size,
        border_bits=args.border,
        out_file=args.out,
        show=args.show
    )