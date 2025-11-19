import pathlib
from typing import Optional
import cv2
import numpy as np
import torch
from mast3r_slam.dataloader import Intrinsics
from mast3r_slam.frame import SharedKeyframes
from mast3r_slam.lietorch_utils import as_SE3
from mast3r_slam.config import config
from mast3r_slam.geometry import constrain_points_to_ray
from plyfile import PlyData, PlyElement


def prepare_savedir(args, dataset):
    save_dir = pathlib.Path("logs")
    if args.save_as != "default":
        save_dir = save_dir / args.save_as
    save_dir.mkdir(exist_ok=True, parents=True)
    seq_name = dataset.dataset_path.stem
    return save_dir, seq_name


def save_traj(
    logdir,
    logfile,
    timestamps,
    frames: SharedKeyframes,
    intrinsics: Optional[Intrinsics] = None,
):
    # log
    logdir = pathlib.Path(logdir)
    logdir.mkdir(exist_ok=True, parents=True)
    logfile = logdir / logfile
    with open(logfile, "w") as f:
        # for keyframe_id in frames.keyframe_ids:
        for i in range(len(frames)):
            keyframe = frames[i]
            t = timestamps[keyframe.frame_id]
            if intrinsics is None:
                T_WC = as_SE3(keyframe.T_WC)
            else:
                T_WC = intrinsics.refine_pose_with_calibration(keyframe)
            x, y, z, qx, qy, qz, qw = T_WC.data.numpy().reshape(-1)
            f.write(f"{t} {x} {y} {z} {qx} {qy} {qz} {qw}\n")


def save_reconstruction(savedir, filename, keyframes, c_conf_threshold):
    savedir = pathlib.Path(savedir)
    savedir.mkdir(exist_ok=True, parents=True)
    pointclouds = []
    colors = []
    for i in range(len(keyframes)):
        keyframe = keyframes[i]
        if config["use_calib"]:
            X_canon = constrain_points_to_ray(
                keyframe.img_shape.flatten()[:2], keyframe.X_canon[None], keyframe.K
            )
            keyframe.X_canon = X_canon.squeeze(0)
        pW = keyframe.T_WC.act(keyframe.X_canon).cpu().numpy().reshape(-1, 3)
        color = (keyframe.uimg.cpu().numpy() * 255).astype(np.uint8).reshape(-1, 3)
        valid = (
            keyframe.get_average_conf().cpu().numpy().astype(np.float32).reshape(-1)
            > c_conf_threshold
        )
        pointclouds.append(pW[valid])
        colors.append(color[valid])
    pointclouds = np.concatenate(pointclouds, axis=0)
    colors = np.concatenate(colors, axis=0)

    save_ply(savedir / filename, pointclouds, colors)


def save_keyframes(savedir, timestamps, keyframes: SharedKeyframes):
    savedir = pathlib.Path(savedir)
    savedir.mkdir(exist_ok=True, parents=True)
    for i in range(len(keyframes)):
        keyframe = keyframes[i]
        t = timestamps[keyframe.frame_id]
        filename = savedir / f"{t}.png"
        cv2.imwrite(
            str(filename),
            cv2.cvtColor(
                (keyframe.uimg.cpu().numpy() * 255).astype(np.uint8), cv2.COLOR_RGB2BGR
            ),
        )


def save_ply(filename, points, colors):
    colors = colors.astype(np.uint8)
    # Combine XYZ and RGB into a structured array
    pcd = np.empty(
        len(points),
        dtype=[
            ("x", "f4"),
            ("y", "f4"),
            ("z", "f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
        ],
    )
    pcd["x"], pcd["y"], pcd["z"] = points.T
    pcd["red"], pcd["green"], pcd["blue"] = colors.T
    vertex_element = PlyElement.describe(pcd, "vertex")
    ply_data = PlyData([vertex_element], text=False)
    ply_data.write(filename)

# 新增: 保存带质量属性的 PLY (lines 108-186)
def save_ply_with_quality(savedir, filename, keyframes, c_conf_threshold, quality_service, patch_size=16):
    savedir = pathlib.Path(savedir)
    savedir.mkdir(exist_ok=True, parents=True)
    points_all, colors_all = [], []
    r_all, dc_all, u_all, cid_all, pri_all = [], [], [], [], []
    for i in range(len(keyframes)):
        kf = keyframes[i]
        if config["use_calib"]:
            X_canon = constrain_points_to_ray(kf.img_shape.flatten()[:2], kf.X_canon[None], kf.K)
            kf.X_canon = X_canon.squeeze(0)
        pW = kf.T_WC.act(kf.X_canon).cpu().numpy().reshape(-1, 3)
        col = (kf.uimg.cpu().numpy() * 255).astype(np.uint8).reshape(-1, 3)
        valid = (kf.get_average_conf().cpu().numpy().astype(np.float32).reshape(-1) > c_conf_threshold)

        H, W = int(kf.img_shape.flatten()[0]), int(kf.img_shape.flatten()[1])
        res = quality_service.get(kf.frame_id) if quality_service is not None else None
        if res is not None:
            #def up(g, mode):
             #   gnp = g.detach().cpu().numpy().astype(np.float32)
            def up(g, mode):
                # Handle both tensor and numpy array
                if torch.is_tensor(g):
                    gnp = g.detach().cpu().numpy().astype(np.float32)
                elif isinstance(g, np.ndarray):
                    gnp = g.astype(np.float32)
                else:
                    gnp = np.array(g, dtype=np.float32)
                gh, gw = gnp.shape[-2], gnp.shape[-1]
                out = cv2.resize(gnp, (W, H), interpolation=cv2.INTER_NEAREST if mode=="nearest" else cv2.INTER_LINEAR)
                return out.reshape(-1)
            dc = up(res["delta_cov"], "linear")
            rr = up(res["r"], "linear")
            uu = up(res["u"], "linear")
            #cc = up(res["class_id"].float(), "nearest").astype(np.uint8)
            if torch.is_tensor(res["class_id"]):
                class_id_float = res["class_id"].float()
            else:
                class_id_float = res["class_id"].astype(np.float32)
            cc = up(class_id_float, "nearest").astype(np.uint8)
            pp = up(res["priority"], "linear")
        else:
            n = H * W
            dc = np.zeros((n,), np.float32)
            rr = np.zeros((n,), np.float32)
            uu = np.zeros((n,), np.float32)
            cc = np.zeros((n,), np.uint8)
            pp = np.zeros((n,), np.float32)

        points_all.append(pW[valid])
        colors_all.append(col[valid])
        r_all.append(rr[valid])
        dc_all.append(dc[valid])
        u_all.append(uu[valid])
        cid_all.append(cc[valid])
        pri_all.append(pp[valid])

    points = np.concatenate(points_all, 0)
    colors = np.concatenate(colors_all, 0)
    r = np.concatenate(r_all, 0).astype(np.float32)
    dc = np.concatenate(dc_all, 0).astype(np.float32)
    u = np.concatenate(u_all, 0).astype(np.float32)
    cid = np.concatenate(cid_all, 0).astype(np.uint8)
    pri = np.concatenate(pri_all, 0).astype(np.float32)

    pcd = np.empty(points.shape[0], dtype=[
        ("x","f4"),("y","f4"),("z","f4"),
        ("red","u1"),("green","u1"),("blue","u1"),
        ("r","f4"),("delta_cov","f4"),("u","f4"),
        ("class_id","u1"),("priority","f4"),
    ])
    pcd["x"], pcd["y"], pcd["z"] = points.T
    pcd["red"], pcd["green"], pcd["blue"] = colors.T
    pcd["r"] = r
    pcd["delta_cov"] = dc
    pcd["u"] = u
    pcd["class_id"] = cid
    pcd["priority"] = pri
    vertex_element = PlyElement.describe(pcd, "vertex")
    PlyData([vertex_element], text=False).write(savedir / filename)
