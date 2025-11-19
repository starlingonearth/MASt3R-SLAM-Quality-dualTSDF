import argparse
import datetime
import pathlib
import sys
import time
import cv2
import lietorch
import torch
import tqdm
import yaml
from mast3r_slam.global_opt import FactorGraph

from mast3r_slam.config import load_config, config, set_global_config
from mast3r_slam.dataloader import Intrinsics, load_dataset
import mast3r_slam.evaluate as eval
from mast3r_slam.frame import Mode, SharedKeyframes, SharedStates, create_frame
from mast3r_slam.mast3r_utils import (
    load_mast3r,
    load_retriever,
    mast3r_inference_mono,
)
from mast3r_slam.multiprocess_utils import new_queue, try_get_msg
from mast3r_slam.tracker import FrameTracker
from mast3r_slam.visualization import WindowMsg, run_visualization
from mast3r_slam.quality import AsynchronousQualityService  # 新增: 导入质量服务
import torch.multiprocessing as mp

def relocalization(frame, keyframes, factor_graph, retrieval_database):
    # we are adding and then removing from the keyframe, so we need to be careful.
    # The lock slows viz down but safer this way...
    with keyframes.lock:
        kf_idx = []
        retrieval_inds = retrieval_database.update(
            frame,
            add_after_query=False,
            k=config["retrieval"]["k"],
            min_thresh=config["retrieval"]["min_thresh"],
        )
        kf_idx += retrieval_inds
        successful_loop_closure = False
        if kf_idx:
            keyframes.append(frame)
            n_kf = len(keyframes)
            kf_idx = list(kf_idx)  # convert to list
            frame_idx = [n_kf - 1] * len(kf_idx)
            print("RELOCALIZING against kf ", n_kf - 1, " and ", kf_idx)
            if factor_graph.add_factors(
                frame_idx,
                kf_idx,
                config["reloc"]["min_match_frac"],
                is_reloc=config["reloc"]["strict"],
            ):
                retrieval_database.update(
                    frame,
                    add_after_query=True,
                    k=config["retrieval"]["k"],
                    min_thresh=config["retrieval"]["min_thresh"],
                )
                print("Success! Relocalized")
                successful_loop_closure = True
                keyframes.T_WC[n_kf - 1] = keyframes.T_WC[kf_idx[0]].clone()
            else:
                keyframes.pop_last()
                print("Failed to relocalize")

        if successful_loop_closure:
            if config["use_calib"]:
                factor_graph.solve_GN_calib()
            else:
                factor_graph.solve_GN_rays()
        return successful_loop_closure

def run_backend(cfg, model, states, keyframes, K, tsdf_global_cfg):  # 新增: 添加 tsdf_global_cfg 参数
    set_global_config(cfg)

    device = keyframes.device
    factor_graph = FactorGraph(model, keyframes, K, device)
    retrieval_database = load_retriever(model)
    # 新增: TSDF 全局管理器初始化 (lines 78-88)
    tsdf_manager = None
    if tsdf_global_cfg.get("enabled", False):
        from mast3r_slam.tsdf import TSDFGlobalManager

        tsdf_manager = TSDFGlobalManager(
            keyframes=keyframes,
            cfg=tsdf_global_cfg,
            use_calib=config.get("use_calib", False),
            device=device,
        )
        tsdf_manager.start()

    mode = states.get_mode()
    while mode is not Mode.TERMINATED:
        mode = states.get_mode()
        if mode == Mode.INIT or states.is_paused():
            time.sleep(0.01)
            continue
        if mode == Mode.RELOC:
            frame = states.get_frame()
            success = relocalization(frame, keyframes, factor_graph, retrieval_database)
            if success:
                states.set_mode(Mode.TRACKING)
            states.dequeue_reloc()
            continue
        idx = -1
        with states.lock:
            if len(states.global_optimizer_tasks) > 0:
                idx = states.global_optimizer_tasks[0]
        if idx == -1:
            time.sleep(0.01)
            continue

        # Graph Construction
        kf_idx = []
        # k to previous consecutive keyframes
        n_consec = 1
        for j in range(min(n_consec, idx)):
            kf_idx.append(idx - 1 - j)
        frame = keyframes[idx]
        retrieval_inds = retrieval_database.update(
            frame,
            add_after_query=True,
            k=config["retrieval"]["k"],
            min_thresh=config["retrieval"]["min_thresh"],
        )
        kf_idx += retrieval_inds

        lc_inds = set(retrieval_inds)
        lc_inds.discard(idx - 1)
        if len(lc_inds) > 0:
            print("Database retrieval", idx, ": ", lc_inds)

        kf_idx = set(kf_idx)  # Remove duplicates by using set
        kf_idx.discard(idx)  # Remove current kf idx if included
        kf_idx = list(kf_idx)  # convert to list
        frame_idx = [idx] * len(kf_idx)
        if kf_idx:
            factor_graph.add_factors(
                kf_idx, frame_idx, config["local_opt"]["min_match_frac"]
            )

        with states.lock:
            states.edges_ii[:] = factor_graph.ii.cpu().tolist()
            states.edges_jj[:] = factor_graph.jj.cpu().tolist()

        if config["use_calib"]:
            factor_graph.solve_GN_calib()
        else:
            factor_graph.solve_GN_rays()

        # 新增: TSDF 全局管理器求解后回调 (lines 149-153)
        if tsdf_manager is not None:
            try:
                tsdf_manager.on_after_backend_solve(factor_graph)
            except Exception as exc:
                print(f"[TSDF-GLOBAL] Optimization failed: {exc}")

        with states.lock:
            if len(states.global_optimizer_tasks) > 0:
                idx = states.global_optimizer_tasks.pop(0)
    # 新增: TSDF 全局管理器关闭 (lines 158-159)
    if tsdf_manager is not None:
        tsdf_manager.shutdown()
if __name__ == "__main__":
    mp.set_start_method("spawn")
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.set_grad_enabled(False)
    device = "cuda:0"
    save_frames = False
    datetime_now = str(datetime.datetime.now()).replace(" ", "_")

    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="datasets/tum/rgbd_dataset_freiburg1_desk")
    parser.add_argument("--config", default="config/base.yaml")
    parser.add_argument("--save-as", default="default")
    parser.add_argument("--no-viz", action="store_true")
    parser.add_argument("--calib", default="")
    args = parser.parse_args()

    load_config(args.config)
    print(args.dataset)
    print(config)

    manager = mp.Manager()
    main2viz = new_queue(manager, args.no_viz)
    viz2main = new_queue(manager, args.no_viz)
    # 新增: 异步质量服务初始化 (line 183)
    quality_service = AsynchronousQualityService(manager=manager)

    dataset = load_dataset(args.dataset)
    dataset.subsample(config["dataset"]["subsample"])
    h, w = dataset.get_img_shape()[0]

    if args.calib:
        with open(args.calib, "r") as f:
            intrinsics = yaml.load(f, Loader=yaml.SafeLoader)
        config["use_calib"] = True
        dataset.use_calibration = True
        dataset.camera_intrinsics = Intrinsics.from_calib(
            dataset.img_size,
            intrinsics["width"],
            intrinsics["height"],
            intrinsics["calibration"],
        )

    keyframes = SharedKeyframes(manager, h, w)
    states = SharedStates(manager, h, w)

    if not args.no_viz:
        viz = mp.Process(
            target=run_visualization,
            args=(config, states, keyframes, main2viz, viz2main),
        )
        viz.start()

    model = load_mast3r(device=device)
    model.share_memory()

    has_calib = dataset.has_calib()
    use_calib = config["use_calib"]

    if use_calib and not has_calib:
        print("[Warning] No calibration provided for this dataset!")
        sys.exit(0)
    K = None
    if use_calib:
        K = torch.from_numpy(dataset.camera_intrinsics.K_frame).to(
            device, dtype=torch.float32
        )
        keyframes.set_intrinsics(K)

    # remove the trajectory from the previous run
    if dataset.save_results:
        save_dir, seq_name = eval.prepare_savedir(args, dataset)
        traj_file = save_dir / f"{seq_name}.txt"
        recon_file = save_dir / f"{seq_name}.ply"
        if traj_file.exists():
            traj_file.unlink()
        if recon_file.exists():
            recon_file.unlink()

    tracker = FrameTracker(model, keyframes, device)
    last_msg = WindowMsg()
    # 新增: 将质量服务附加到跟踪器 (line 243)
    # 注释: 将质量服务附加到跟踪器
    tracker.quality_service = quality_service

    # 新增: TSDF 精化器初始化部分 (lines 245-284)
    # 注释: TSDF 精化器初始化
    # ------------------------------
    # TSDF Refiner initialization
    # ------------------------------
    tsdf_refiner = None
    tsdf_enabled = config.get("tsdf_refine", {}).get("enabled", False)
    tsdf_cfg = config.get("tsdf_refine", {})

    # Shutdown parameters (can be overridden in config)
    MAX_SHUTDOWN_WAIT_S  = float(tsdf_cfg.get("max_shutdown_wait_s", -1.0))  # <=0  unlimited
    MIN_SHUTDOWN_WAIT_S  = float(tsdf_cfg.get("min_shutdown_wait_s", 0.0))
    DEFAULT_BLOCK_TIME_S = float(tsdf_cfg.get("default_block_time_s", 12.0))
    PROGRESS_STALL_S     = float(tsdf_cfg.get("progress_stall_s", 45.0))
    GRACE_EMPTY_S        = float(tsdf_cfg.get("grace_empty_s", 3.0))

    UNLIMITED_WAIT = MAX_SHUTDOWN_WAIT_S <= 0

    if tsdf_enabled:
        print("[MAIN] Initializing TSDF Refiner...")
        print(f"[MAIN] TSDF config: {tsdf_cfg}")
        try:
            from mast3r_slam.tsdf_refine import TSDFRefiner
            tsdf_refiner = TSDFRefiner(
                cfg=tsdf_cfg,
                shared_keyframes=keyframes,
                quality_service=quality_service,
                device=device,
            )
            tsdf_refiner.start()
            if tsdf_refiner.is_alive():
                print("[MAIN] TSDF refiner thread started successfully")
            else:
                print("[MAIN] WARNING: TSDF refiner thread failed to start")
                tsdf_refiner = None
        except Exception as e:
            print(f"[MAIN] Failed to initialize TSDF refiner: {e}")
            import traceback
            traceback.print_exc()
            tsdf_refiner = None
    else:
        print("[MAIN] TSDF refinement disabled in config")

    # 新增: 后端进程现在包含 TSDF 全局配置参数 (lines 282-293)
    backend = mp.Process(
        target=run_backend,
        args=(
            config,
            model,
            states,
            keyframes,
            K,
            config.get("tsdf_global", {}),  # 新增: TSDF 全局配置
        ),
    )
    backend.start()

    # 新增: 主循环部分，包含 TSDF 跟踪变量 (lines 304-322)
    # ------------------------------
    # Main loop
    # ------------------------------
    i = 0
    fps_timer = time.time()
    frames = []

    tsdf_total_scheduled = 0
    tsdf_last_success_count = 0
    tsdf_shutdown_in_progress = False  # prevent scheduling during shutdown
    final_pass_scheduled_once = False  # idempotency guard

    def _queue_size_safe(refiner):
        try:
            return refiner.queue.qsize()
        except Exception:
            return None

    print("Starting main processing loop...")
    while True:
        mode = states.get_mode()
        msg = try_get_msg(viz2main)
        last_msg = msg if msg is not None else last_msg
        if last_msg.is_terminated:
            print("Termination requested")
            states.set_mode(Mode.TERMINATED)
            break

        if last_msg.is_paused and not last_msg.next:
            states.pause()
            time.sleep(0.01)
            continue

        if not last_msg.is_paused:
            states.unpause()

        if i == len(dataset):
            print(f"Processed all {len(dataset)} frames")
            states.set_mode(Mode.TERMINATED)
            break

        timestamp, img = dataset[i]
        if save_frames:
            frames.append(img)

        # last camera pose for the frame
        T_WC = (
            lietorch.Sim3.Identity(1, device=device)
            if i == 0
            else states.get_frame().T_WC
        )
        frame = create_frame(i, img, T_WC, img_size=dataset.img_size, device=device)

        if mode == Mode.INIT:
            X_init, C_init = mast3r_inference_mono(model, frame)
            frame.update_pointmap(X_init, C_init)
            keyframes.append(frame)
            states.queue_global_optimization(len(keyframes) - 1)
            states.set_mode(Mode.TRACKING)
            states.set_frame(frame)
            i += 1
            continue

        if mode == Mode.TRACKING:
            add_new_kf, match_info, try_reloc = tracker.track(frame)
            if try_reloc:
                states.set_mode(Mode.RELOC)
            states.set_frame(frame)

        elif mode == Mode.RELOC:
            X, C = mast3r_inference_mono(model, frame)
            frame.update_pointmap(X, C)
            states.set_frame(frame)
            states.queue_reloc()
            # 新增: 单线程模式等待重定位 (lines 378-382)
            while config["single_thread"]:
                with states.lock:
                    if states.reloc_sem.value == 0:
                        break
                time.sleep(0.01)

        else:
            raise Exception("Invalid mode")

        if add_new_kf:
            keyframes.append(frame)
            states.queue_global_optimization(len(keyframes) - 1)

            # 新增: 单线程模式等待后端 (lines 391-395)
            while config["single_thread"]:
                with states.lock:
                    if len(states.global_optimizer_tasks) == 0:
                        break
                time.sleep(0.01)

            # 新增: TSDF 精化调度逻辑 (lines 397-416)
            # 仅在未关闭且精化器健康时调度 TSDF 精化
            sf = getattr(tsdf_refiner, "stop_flag", None)
            if (
                tsdf_refiner is not None
                and tsdf_refiner.is_alive()
                and not tsdf_shutdown_in_progress
                and not (sf is not None and sf.is_set())
            ):
                current_kf_id = len(keyframes) - 1
                old_queue_size = _queue_size_safe(tsdf_refiner)
                try:
                    tsdf_refiner.maybe_schedule_sliding_window(current_kf_id)
                except Exception as e:
                    print(f"[TSDF] Scheduling error: {e}")
                new_queue_size = _queue_size_safe(tsdf_refiner)
                if old_queue_size is not None and new_queue_size is not None:
                    if new_queue_size > old_queue_size:
                        blocks_added = new_queue_size - old_queue_size
                        tsdf_total_scheduled += blocks_added
                        print(f"[TSDF] Scheduled {blocks_added} blocks for keyframe {current_kf_id}")

        # 新增: 增强的状态日志，包含 TSDF 统计信息 (lines 418-438)
        if i % 30 == 0:
            FPS = i / (time.time() - fps_timer + 1e-6)
            status_msg = f"[STATUS] Frame {i}, FPS: {FPS:.2f}"

            if tsdf_refiner is not None and tsdf_refiner.is_alive():
                try:
                    stats = tsdf_refiner.stats
                    success_count = stats.get("successful_blocks", 0)
                    total_count = stats.get("total_blocks", 0)
                    qsz = _queue_size_safe(tsdf_refiner)
                    if success_count > tsdf_last_success_count:
                        tsdf_last_success_count = success_count
                        print(f"[TSDF] New successes! Total: {success_count}")
                    if total_count > 0:
                        success_rate = success_count / max(1, total_count)
                        qinfo = f", Queue: {qsz}" if qsz is not None else ""
                        status_msg += f", TSDF: {success_count}/{total_count} ({success_rate:.1%}){qinfo}"
                except Exception:
                    pass

            print(status_msg)

        i += 1

    print(f"\nMain loop completed. Processed {i} frames.")

    # 新增: TSDF 精化器关闭部分 (lines 444-557)
    # ------------------------------
    # TSDF shutdown: schedule final pass drain queue until empty  print stats  stop
    # ------------------------------
    if tsdf_refiner is not None:
        tsdf_shutdown_in_progress = True  # block any further scheduling

        print("\n" + "=" * 60)
        print("TSDF REFINER SHUTDOWN")
        print("=" * 60)
        try:
            # (1) schedule final pass once
            if len(keyframes) > 0 and not final_pass_scheduled_once:
                final_kf_id = len(keyframes) - 1
                print(f"[TSDF] Triggering final pass for {final_kf_id} keyframes...")
                try:
                    tsdf_refiner.schedule_final_pass(final_kf_id)
                except Exception as e:
                    print(f"[TSDF] schedule_final_pass error: {e}")
                final_pass_scheduled_once = True

            # (2) drain until queue is empty (time-unlimited if configured)
            #     we also allow a tiny grace to cover the last in-flight block after qsize hits zero
            start = time.time()
            last_report = start
            last_success = int(getattr(tsdf_refiner, "stats", {}).get("successful_blocks", 0))

            if MIN_SHUTDOWN_WAIT_S > 0:
                time.sleep(MIN_SHUTDOWN_WAIT_S)

            while True:
                qsz = _queue_size_safe(tsdf_refiner)
                stats = getattr(tsdf_refiner, "stats", {}) or {}
                success = int(stats.get("successful_blocks", 0))

                # periodic progress print
                if time.time() - last_report >= 5.0:
                    print(f"[TSDF] Draining... queue={qsz}, successes={success}")
                    last_report = time.time()

                # if queue size known and empty, give a short grace and break
                if qsz is not None and qsz == 0:
                    # if success increased right at empty, refresh timer
                    if success > last_success:
                        last_success = success
                        last_report = time.time()
                    # grace for last in-flight block to write back
                    time.sleep(GRACE_EMPTY_S)
                    # re-check: still empty?
                    qsz2 = _queue_size_safe(tsdf_refiner)
                    if qsz2 == 0:
                        print("[TSDF] Queue drained.")
                        break

                # if we do NOT know queue size (shouldn't happen with threads), fallback to progress-based wait
                if qsz is None:
                    if not UNLIMITED_WAIT and (time.time() - start) >= MAX_SHUTDOWN_WAIT_S:
                        print("[TSDF] Reached shutdown wait budget (fallback path).")
                        break
                    # keep waiting as long as progress happens within PROGRESS_STALL_S
                    time.sleep(0.5)
                    if success > last_success:
                        last_success = success
                        last_report = time.time()
                    elif (time.time() - last_report) >= PROGRESS_STALL_S:
                        print(f"[TSDF] No progress for {PROGRESS_STALL_S:.1f}s (fallback); assuming drained.")
                        break
                else:
                    # known queue size and not empty: keep waiting (unlimited if configured)
                    if not UNLIMITED_WAIT and (time.time() - start) >= MAX_SHUTDOWN_WAIT_S:
                        print("[TSDF] Reached shutdown wait budget.")
                        break
                    time.sleep(0.5)

            # (3) final statistics
            if hasattr(tsdf_refiner, "stats"):
                stats = tsdf_refiner.stats
                success_count = stats.get("successful_blocks", 0)
                total_count = stats.get("total_blocks", 0)
                print(f"[TSDF] Final Statistics:")
                print(f"  - Total blocks processed: {total_count}")
                print(f"  - Successful refinements: {success_count}")
                if total_count > 0:
                    success_rate = success_count / max(1, total_count)
                    avg_time = stats.get("total_processing_time", 0.0) / max(1, total_count)
                    print(f"  - Success rate: {success_rate:.1%}")
                    print(f"  - Average time per block: {avg_time:.3f}s")
                debug_info = stats.get("debug_info", {}) or {}
                print(f"  - TSDF constructions: {debug_info.get('tsdf_constructions', 0)}")
                print(f"  - Surface extractions: {debug_info.get('surface_extractions', 0)}")
                print(f"  - Displacement rejects: {debug_info.get('displacement_rejects', 0)}")
                print(f"  - Hit ratio rejects: {debug_info.get('hit_ratio_rejects', 0)}")

            # (4) stop thread
            print("[TSDF] Stopping refiner thread...")
            try:
                tsdf_refiner.stop_flag.set()
            except Exception:
                pass
            # unlimited join if unlimited wait requested
            tsdf_refiner.join(timeout=None if UNLIMITED_WAIT else 60.0)
            if tsdf_refiner.is_alive():
                print("[TSDF] WARNING: Thread did not stop cleanly")
            else:
                print("[TSDF] Shutdown completed successfully")

            # (5) summary
            if hasattr(tsdf_refiner, "stats"):
                final_success = tsdf_refiner.stats.get("successful_blocks", 0)
                if final_success > 0:
                    print(f"[TSDF] ? Successfully refined {final_success} blocks!")
                else:
                    print("[TSDF] No successful refinements in this run")

        except Exception as e:
            print(f"[TSDF] Shutdown error: {e}")
            import traceback
            traceback.print_exc()

        print("=" * 60)

    # 新增: 增强的保存结果部分，包含质量 PLY 导出 (lines 570-593)
    # ------------------------------
    # Save results
    # ------------------------------
    if dataset.save_results:
        print("\nSaving results...")
        save_dir, seq_name = eval.prepare_savedir(args, dataset)
        eval.save_traj(save_dir, f"{seq_name}.txt", dataset.timestamps, keyframes)
        eval.save_reconstruction(
            save_dir,
            f"{seq_name}.ply",
            keyframes,
            0.0,  # 新增: 从 last_msg.C_conf_threshold 改为 0.0
        )
        # 新增: 保存带质量属性的 PLY (lines 583-589)
        eval.save_ply_with_quality(
            save_dir,
            f"{seq_name}_quality.ply",
            keyframes,
            0.0,
            quality_service,
        )
        eval.save_keyframes(
            save_dir / "keyframes" / seq_name, dataset.timestamps, keyframes
        )
        print(f"Results saved to {save_dir}")

    if save_frames:
        print("Saving frames...")
        savedir = pathlib.Path(f"logs/frames/{datetime_now}")
        savedir.mkdir(exist_ok=True, parents=True)
        for i, frame in tqdm.tqdm(enumerate(frames), total=len(frames)):
            frame = (frame * 255).clip(0, 255)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imwrite(f"{savedir}/{i}.png", frame)
        print(f"Frames saved to {savedir}")

    # 新增: 增强的关闭序列，包含质量服务 (lines 605-614)
    print("\nShutting down services...")
    quality_service.shutdown()
    backend.join()

    if not args.no_viz:
        viz.join()

    print("\n" + "=" * 60)
    print("ALL PROCESSES SHUTDOWN COMPLETE")
    print("=" * 60)
