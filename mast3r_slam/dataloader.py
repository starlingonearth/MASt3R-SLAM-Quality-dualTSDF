import pathlib
import re
import cv2
from natsort import natsorted
import numpy as np
import torch
import pyrealsense2 as rs
import yaml
import json  # 新增: 用于 ReplicaDataset cam_params.json 解析
import os    # 新增: 用于 load_dataset 路径扩展名处理

from mast3r_slam.mast3r_utils import resize_img
from mast3r_slam.config import config

HAS_TORCHCODEC = True
try:
    from torchcodec.decoders import VideoDecoder
except Exception as e:
    HAS_TORCHCODEC = False


class MonocularDataset(torch.utils.data.Dataset):
    def __init__(self, dtype=np.float32):
        self.dtype = dtype
        self.rgb_files = []
        self.timestamps = []
        self.img_size = 512
        self.camera_intrinsics = None
        self.use_calibration = config["use_calib"]
        self.save_results = True

    def __len__(self):
        return len(self.rgb_files)

    def __getitem__(self, idx):
        # Call get_image before timestamp for realsense camera
        img = self.get_image(idx)
        timestamp = self.get_timestamp(idx)
        return timestamp, img

    def get_timestamp(self, idx):
        return self.timestamps[idx]

    def read_img(self, idx):
        img = cv2.imread(str(self.rgb_files[idx]))  # 新增: 添加 str() 以兼容 pathlib
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    def get_image(self, idx):
        img = self.read_img(idx)
        if self.use_calibration:
            img = self.camera_intrinsics.remap(img)
        return img.astype(self.dtype) / 255.0

    def get_img_shape(self):
        img = self.read_img(0)
        raw_img_shape = img.shape
        img = resize_img(img, self.img_size)
        # 3XHxW, HxWx3 -> HxW, HxW
        return img["img"][0].shape[1:], raw_img_shape[:2]

    def subsample(self, subsample):
        self.rgb_files = self.rgb_files[::subsample]
        self.timestamps = self.timestamps[::subsample]

    def has_calib(self):
        return self.camera_intrinsics is not None


class TUMDataset(MonocularDataset):
    def __init__(self, dataset_path):
        super().__init__()
        self.dataset_path = pathlib.Path(dataset_path)
        rgb_list = self.dataset_path / "rgb.txt"
        tstamp_rgb = np.loadtxt(rgb_list, delimiter=" ", dtype=np.unicode_, skiprows=0)
        self.rgb_files = [self.dataset_path / f for f in tstamp_rgb[:, 1]]
        self.timestamps = tstamp_rgb[:, 0]

        match = re.search(r"freiburg(\d+)", dataset_path)
        idx = int(match.group(1))
        if idx == 1:
            calib = np.array(
                [517.3, 516.5, 318.6, 255.3, 0.2624, -0.9531, -0.0054, 0.0026, 1.1633]
            )
        if idx == 2:
            calib = np.array(
                [520.9, 521.0, 325.1, 249.7, 0.2312, -0.7849, -0.0033, -0.0001, 0.9172]
            )
        if idx == 3:
            calib = np.array([535.4, 539.2, 320.1, 247.6])
        W, H = 640, 480
        self.camera_intrinsics = Intrinsics.from_calib(self.img_size, W, H, calib)


class EurocDataset(MonocularDataset):
    def __init__(self, dataset_path):
        super().__init__()
        # For Euroc dataset, the distortion is too much to handle for MASt3R.
        # So we always undistort the images, but the calibration will not be used for any later optimization unless specified.
        self.use_calibration = True
        self.dataset_path = pathlib.Path(dataset_path)
        rgb_list = self.dataset_path / "mav0/cam0/data.csv"
        tstamp_rgb = np.loadtxt(rgb_list, delimiter=",", dtype=np.unicode_, skiprows=0)
        self.rgb_files = [
            self.dataset_path / "mav0/cam0/data" / f for f in tstamp_rgb[:, 1]
        ]
        self.timestamps = tstamp_rgb[:, 0]
        with open(self.dataset_path / "mav0/cam0/sensor.yaml") as f:
            self.cam0 = yaml.load(f, Loader=yaml.FullLoader)
        W, H = self.cam0["resolution"]
        intrinsics = self.cam0["intrinsics"]
        distortion = np.array(self.cam0["distortion_coefficients"])
        self.camera_intrinsics = Intrinsics.from_calib(
            self.img_size, W, H, [*intrinsics, *distortion], always_undistort=True
        )

    def read_img(self, idx):
        img = cv2.imread(str(self.rgb_files[idx]), cv2.IMREAD_GRAYSCALE)  # 新增: 添加 str() 以兼容 pathlib
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


class ETH3DDataset(MonocularDataset):
    def __init__(self, dataset_path):
        super().__init__()
        self.dataset_path = pathlib.Path(dataset_path)
        rgb_list = self.dataset_path / "rgb.txt"
        tstamp_rgb = np.loadtxt(rgb_list, delimiter=" ", dtype=np.unicode_, skiprows=0)
        self.rgb_files = [self.dataset_path / f for f in tstamp_rgb[:, 1]]
        self.timestamps = tstamp_rgb[:, 0]
        calibration = np.loadtxt(
            self.dataset_path / "calibration.txt",
            delimiter=" ",
            dtype=np.float32,
            skiprows=0,
        )
        _, (H, W) = self.get_img_shape()
        self.camera_intrinsics = Intrinsics.from_calib(self.img_size, W, H, calibration)


class SevenScenesDataset(MonocularDataset):
    def __init__(self, dataset_path):
        super().__init__()
        self.dataset_path = pathlib.Path(dataset_path)
        self.rgb_files = natsorted(
            list((self.dataset_path / "seq-01").glob("*.color.png"))
        )
        self.timestamps = np.arange(0, len(self.rgb_files)).astype(self.dtype)
        fx, fy, cx, cy = 585.0, 585.0, 320.0, 240.0
        self.camera_intrinsics = Intrinsics.from_calib(
            self.img_size, 640, 480, [fx, fy, cx, cy]
        )


class RealsenseDataset(MonocularDataset):
    def __init__(self):
        super().__init__()
        self.dataset_path = None
        self.pipeline = rs.pipeline()
        # self.h, self.w = 720, 1280
        self.h, self.w = 480, 640
        self.rs_config = rs.config()
        self.rs_config.enable_stream(
            rs.stream.color, self.w, self.h, rs.format.bgr8, 30
        )
        self.profile = self.pipeline.start(self.rs_config)

        self.rgb_sensor = self.profile.get_device().query_sensors()[1]
        # self.rgb_sensor.set_option(rs.option.enable_auto_exposure, False)
        # self.rgb_sensor.set_option(rs.option.enable_auto_white_balance, False)
        # self.rgb_sensor.set_option(rs.option.exposure, 200)
        self.rgb_profile = rs.video_stream_profile(
            self.profile.get_stream(rs.stream.color)
        )
        self.save_results = False

        if self.use_calibration:
            rgb_intrinsics = self.rgb_profile.get_intrinsics()
            self.camera_intrinsics = Intrinsics.from_calib(
                self.img_size,
                self.w,
                self.h,
                [
                    rgb_intrinsics.fx,
                    rgb_intrinsics.fy,
                    rgb_intrinsics.ppx,
                    rgb_intrinsics.ppy,
                ],
            )

    def __len__(self):
        return 999999

    def get_timestamp(self, idx):
        return self.timestamps[idx]

    def read_img(self, idx):
        frameset = self.pipeline.wait_for_frames()
        timestamp = frameset.get_timestamp()
        timestamp /= 1000
        self.timestamps.append(timestamp)

        rgb_frame = frameset.get_color_frame()
        img = np.asanyarray(rgb_frame.get_data())
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(self.dtype)
        return img


class Webcam(MonocularDataset):
    def __init__(self):
        super().__init__()
        self.use_calibration = False
        self.dataset_path = None
        # load webcam using opencv
        self.cap = cv2.VideoCapture(-1)
        self.save_results = False

    def __len__(self):
        return 999999

    def get_timestamp(self, idx):
        return self.timestamps[idx]

    def read_img(self, idx):
        ret, img = self.cap.read()
        if not ret:
            raise ValueError("Failed to read image")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.timestamps.append(idx / 30)

        return img


class MP4Dataset(MonocularDataset):
    def __init__(self, dataset_path):
        super().__init__()
        self.use_calibration = False
        self.dataset_path = pathlib.Path(dataset_path)
        if HAS_TORCHCODEC:
            self.decoder = VideoDecoder(str(self.dataset_path))
            self.fps = self.decoder.metadata.average_fps
            self.total_frames = self.decoder.metadata.num_frames
        else:
            print("torchcodec is not installed. This may slow down the dataloader")
            self.cap = cv2.VideoCapture(str(self.dataset_path))
            self.fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

        self.stride = config["dataset"]["subsample"]

    def __len__(self):
        return self.total_frames // self.stride

    def read_img(self, idx):
        if HAS_TORCHCODEC:
            img = self.decoder[idx * self.stride]  # c,h,w
            img = img.permute(1, 2, 0)
            img = img.numpy()
        else:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, idx * self.stride)
            ret, img = self.cap.read()
            if not ret:
                raise ValueError("Failed to read image")
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(self.dtype)
        timestamp = idx / self.fps
        self.timestamps.append(timestamp)
        return img


class RGBFiles(MonocularDataset):
    def __init__(self, dataset_path):
        super().__init__()
        self.use_calibration = False
        self.dataset_path = pathlib.Path(dataset_path)
        self.rgb_files = natsorted(list((self.dataset_path).glob("*.png")))
        self.timestamps = np.arange(0, len(self.rgb_files)).astype(self.dtype) / 30.0

# 新增: ReplicaDataset 类 (lines 275-468)
class ReplicaDataset(MonocularDataset):
    """
    Dataset adapter for the Replica layout:

        <...>/datasets/Replica/
          cam_params.json
          office0/
            results/
              frame000000.jpg
              frame000001.jpg
              ...
              depth000000.png
              ...
            traj.txt  # optional (first column = timestamp)
          office1/
          room0/
          ...

    Notes:
    - Only color frames are used (frame*.jpg / frame*.png). Depth files are ignored.
    - Timestamps: prefer traj.txt first column; otherwise synthesize 30 FPS.
    - Intrinsics: try cam_params.json (supports multiple key layouts). If not
      available, a safe default is used (f ~ 0.9*W, cx = W/2, cy = H/2).
    """

    def __init__(self, dataset_path: str):
        super().__init__()
        self.dataset_path = pathlib.Path(dataset_path)

        # -------- 1) Resolve image directory --------
        img_dir = self.dataset_path / "results"
        if not img_dir.is_dir():
            if self.dataset_path.name.lower() == "results":
                img_dir = self.dataset_path
            else:
                img_dir = self.dataset_path  # fallback: allow pointing to sequence root

        # Prefer results/color or results/rgb if they exist
        candidate_dirs = []
        for sub in ("color", "rgb"):
            p = img_dir / sub
            if p.is_dir():
                candidate_dirs.append(p)
        candidate_dirs.append(img_dir)

        # -------- 2) Collect COLOR images only (exclude depth) --------
        # Accept frame*.jpg/jpeg/png (case-insensitive). Do NOT grab generic *.png
        # to avoid swallowing depth*.png.
        patterns = [
            "frame*.jpg", "frame*.JPG",
            "frame*.jpeg", "frame*.JPEG",
            "frame*.png", "frame*.PNG",
        ]

        rgb_paths = []
        # Non-recursive first (fast path)
        for d in candidate_dirs:
            for pat in patterns:
                rgb_paths += list(d.glob(pat))
            if rgb_paths:
                break

        # If nothing found, try recursive search (handles nested layouts)
        if not rgb_paths:
            for d in candidate_dirs:
                for pat in patterns:
                    rgb_paths += list(d.rglob(pat))

        # Extra safety: exclude anything that looks like depth
        rgb_paths = [p for p in rgb_paths if "depth" not in p.name.lower()]

        # Deduplicate and natural sort
        self.rgb_files = [str(p) for p in natsorted(set(rgb_paths))]


        if len(self.rgb_files) == 0:
            raise FileNotFoundError(
                f"[ReplicaDataset] No RGB frames (frame*.jpg/png) found under {img_dir} (and subdirs)."
            )

        # -------- 3) Determine image size from the first frame --------
        img0 = cv2.imread(str(self.rgb_files[0]))
        if img0 is None:
            raise RuntimeError(f"[ReplicaDataset] Failed to read image: {self.rgb_files[0]}")
        H, W = img0.shape[:2]

        # -------- 4) Build timestamps (traj.txt preferred) --------
        ts = None
        traj_file = self.dataset_path / "traj.txt"
        if traj_file.is_file():
            try:
                arr = np.loadtxt(str(traj_file), dtype=np.float64)
                if arr.ndim == 1:
                    arr = arr.reshape(-1, 1)
                # Typical layout: ts tx ty tz qx qy qz qw; take the first column
                if arr.shape[0] >= len(self.rgb_files):
                    ts = arr[: len(self.rgb_files), 0].astype(self.dtype)
            except Exception:
                ts = None

        if ts is None:
            # Fallback: 30 FPS synthetic timestamps
            ts = np.arange(len(self.rgb_files), dtype=self.dtype) / 30.0

        self.timestamps = ts

        # -------- 5) Camera intrinsics from cam_params.json (if available) --------
        cam_json = None
        for p in (
            self.dataset_path / "cam_params.json",
            self.dataset_path.parent / "cam_params.json",
            self.dataset_path.parent.parent / "cam_params.json",
        ):
            if p.exists():
                cam_json = p
                break

        fx = fy = cx = cy = None
        distortion = None

        if cam_json is not None:
            try:
                with open(cam_json, "r", encoding="utf-8") as f:
                    cam = json.load(f)
                fx, fy, cx, cy, distortion = self._parse_cam_dict(cam)
                if fx is None and isinstance(cam, dict):
                    inner = cam.get("camera") or cam.get("rgb") or cam.get("color")
                    if isinstance(inner, dict):
                        fx, fy, cx, cy, distortion = self._parse_cam_dict(inner)
            except Exception:
                # Keep defaults below if parsing fails
                pass

        if fx is not None and fy is not None and cx is not None and cy is not None:
            calib = [float(fx), float(fy), float(cx), float(cy)]
            if distortion:
                calib += [float(v) for v in distortion]
            self.camera_intrinsics = Intrinsics.from_calib(self.img_size, W, H, calib)
        else:
            # Safe default if calibration not found. If config.use_calib=False,
            # the runtime will ignore intrinsics anyway.
            self.camera_intrinsics = Intrinsics.from_calib(
                self.img_size, W, H, [0.9 * W, 0.9 * W, W / 2.0, H / 2.0]
            )

    # -------- helpers --------

    @staticmethod
    def _parse_cam_dict(d):
        """
        Parse common JSON layouts:
          - {"fx":..., "fy":..., "cx":..., "cy":..., "distortion":[...]} (or k1/k2/p1/p2/k3)
          - {"intrinsics":[fx, fy, cx, cy], "distortion":[...]}
          - {"K":[[fx,0,cx],[0,fy,cy],[0,0,1]]} or {"K":[fx,0,cx, 0,fy,cy, 0,0,1]}
        Returns (fx, fy, cx, cy, distortion_list_or_None), with None on failure.
        """
        fx = fy = cx = cy = None
        dist = None

        if not isinstance(d, dict):
            return fx, fy, cx, cy, dist

        # Mode 1: separate scalar keys
        if all(k in d for k in ("fx", "fy", "cx", "cy")):
            fx, fy, cx, cy = float(d["fx"]), float(d["fy"]), float(d["cx"]), float(d["cy"])
            if "distortion" in d and isinstance(d["distortion"], (list, tuple)):
                dist = list(map(float, d["distortion"]))
            else:
                ks = []
                for k in ("k1", "k2", "p1", "p2", "k3"):
                    if k in d:
                        ks.append(float(d[k]))
                dist = ks if ks else None
            return fx, fy, cx, cy, dist

        # Mode 2: intrinsics vector
        intr = d.get("intrinsics")
        if isinstance(intr, (list, tuple)) and len(intr) >= 4:
            fx, fy, cx, cy = float(intr[0]), float(intr[1]), float(intr[2]), float(intr[3])
            if "distortion" in d and isinstance(d["distortion"], (list, tuple)):
                dist = list(map(float, d["distortion"]))
            return fx, fy, cx, cy, dist

        # Mode 3: K matrix (3x3 or flat-9)
        K = d.get("K")
        if isinstance(K, (list, tuple)):
            if len(K) == 9:
                fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
                return fx, fy, cx, cy, dist
            if len(K) == 3 and isinstance(K[0], (list, tuple)) and len(K[0]) == 3:
                fx, fy, cx, cy = float(K[0][0]), float(K[1][1]), float(K[0][2]), float(K[1][2])
                return fx, fy, cx, cy, dist

        return fx, fy, cx, cy, dist



class Intrinsics:
    def __init__(self, img_size, W, H, K_orig, K, distortion, mapx, mapy):
        self.img_size = img_size
        self.W, self.H = W, H
        self.K_orig = K_orig
        self.K = K
        self.distortion = distortion
        self.mapx = mapx
        self.mapy = mapy
        _, (scale_w, scale_h, half_crop_w, half_crop_h) = resize_img(
            np.zeros((H, W, 3)), self.img_size, return_transformation=True
        )
        self.K_frame = self.K.copy()
        self.K_frame[0, 0] = self.K[0, 0] / scale_w
        self.K_frame[1, 1] = self.K[1, 1] / scale_h
        self.K_frame[0, 2] = self.K[0, 2] / scale_w - half_crop_w
        self.K_frame[1, 2] = self.K[1, 2] / scale_h - half_crop_h

    def remap(self, img):
        return cv2.remap(img, self.mapx, self.mapy, cv2.INTER_LINEAR)

    @staticmethod
    def from_calib(img_size, W, H, calib, always_undistort=False):
        if not config["use_calib"] and not always_undistort:
            return None
        fx, fy, cx, cy = calib[:4]
        distortion = np.zeros(4)
        if len(calib) > 4:
            distortion = np.array(calib[4:])
        K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        K_opt = K.copy()
        mapx, mapy = None, None
        center = config["dataset"]["center_principle_point"]
        K_opt, _ = cv2.getOptimalNewCameraMatrix(
            K, distortion, (W, H), 0, (W, H), centerPrincipalPoint=center
        )
        mapx, mapy = cv2.initUndistortRectifyMap(
            K, distortion, None, K_opt, (W, H), cv2.CV_32FC1
        )

        return Intrinsics(img_size, W, H, K, K_opt, distortion, mapx, mapy)

# 新增: 重构的 load_dataset 函数 (lines 514-536)
def load_dataset(dataset_path: str):
    tokens = [s.lower() for s in re.split(r'[\\/]+', dataset_path)]
    # match Replica first
    if 'replica' in tokens:
        return ReplicaDataset(dataset_path)

    if 'tum' in tokens:
        return TUMDataset(dataset_path)
    if 'euroc' in tokens:
        return EurocDataset(dataset_path)
    if 'eth3d' in tokens:
        return ETH3DDataset(dataset_path)
    if '7-scenes' in tokens or '7scenes' in tokens or '7_scenes' in tokens:
        return SevenScenesDataset(dataset_path)
    if 'realsense' in tokens:
        return RealsenseDataset()
    if 'webcam' in tokens:
        return Webcam()

    ext = os.path.splitext(dataset_path)[1].lower()
    if ext in ['.mp4', '.avi', '.mov']:
        return MP4Dataset(dataset_path)
    return RGBFiles(dataset_path)