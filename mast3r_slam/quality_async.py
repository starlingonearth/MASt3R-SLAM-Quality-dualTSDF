import time
import queue
import threading
import torch
import numpy as np 
from collections import deque
from mast3r_slam.config import config
from mast3r_slam.quality_core import compute_batch

# 改用threading而非multiprocessing，以支持GPU加速

def _prep_job(j, dev):
    """Prepare job for processing"""
    if "__quit__" in j: 
        return j
        
    out = {
        "kf_id": int(j["kf_id"]),
        "frame_id": int(j.get("frame_id", j["kf_id"])),  # 支持frame_id
        "H": int(j["H"]),
        "W": int(j["W"]),
        "t_norm": torch.as_tensor(j["t_norm"], device=dev, dtype=torch.float32),
        "theta": torch.as_tensor(j["theta"], device=dev, dtype=torch.float32)
    }
    
    def tens(x, dtype=torch.float32):
        if x is None: 
            return None
        if isinstance(x, np.ndarray):
            t = torch.from_numpy(x)
        elif torch.is_tensor(x):
            t = x
        else:
            t = torch.as_tensor(x)
        #t = x if torch.is_tensor(x) else torch.as_tensor(x)
        if t.dtype == torch.bool: 
            return t.to(dev)
        return t.to(device=dev, dtype=dtype)
    
    out["valid_kf"] = tens(j["valid_kf"], torch.bool)
    out["r_pix"] = tens(j["r_pix"]) if j.get("r_pix") is not None else torch.zeros(out["H"]*out["W"], device=dev)
    out["Ck"] = tens(j["Ck"])
    out["Qk"] = tens(j["Qk"])
    ew = j.get("cov_ewma", None)
    out["cov_ewma"] = tens(ew) if ew is not None else None
    return out

class AsynchronousQualityService:
    """
    Fixed Quality Service:
    - 改用threading而非multiprocessing，支持GPU加速
    - 添加EWMA状态持久化
    - 双索引cache（kf_id和frame_id）
    - 回调通知机制
    - 全局统计支持
    """
    def __init__(self, manager=None, device=None):
        # 使用普通queue而非multiprocessing queue
        self.job_q = queue.Queue(maxsize=100)
        self.res_q = queue.Queue(maxsize=100)
        
        # 双索引cache：支持kf_id和frame_id查询
        self.cache_by_kf_id = {}
        self.cache_by_frame_id = {}
        
        # EWMA状态持久化：kf_id -> ewma tensor
        self.ewma_state = {}
        
        # 回调通知机制
        self.callbacks = []
        self.callback_lock = threading.Lock()
        
        # 全局统计（滑动窗口）
        self.global_stats = {
            "r_median": 1.0, "r_mad": 0.5,
            "u_median": 0.5, "u_mad": 0.2
        }
        self.stats_window = deque(maxlen=50)
        self.stats_lock = threading.Lock()
        
        # 配置参数
        qcfg = config.get("quality", {})
        self.cfg = {
            "patch_size": int(qcfg.get("patch_size", 16)),
            "batch_size": int(qcfg.get("batch_size", 4)),
            "alpha": float(qcfg.get("metrics", {}).get("coverage", {}).get("alpha_ema", 0.8)),
            "b0": float(qcfg.get("metrics", {}).get("coverage", {}).get("b0", 0.15)),
            "theta0": float(qcfg.get("metrics", {}).get("coverage", {}).get("theta0_deg", 10.0)) * (3.1415926535/180.0),
            "C_thr": float(config.get("tracking", {}).get("C_conf", 0.0)),
            "Q_thr": float(config.get("tracking", {}).get("Q_conf", 0.0)),
            "tzr": float(qcfg.get("thresholds", {}).get("z_r", 1.0)),
            "tzu": float(qcfg.get("thresholds", {}).get("z_u", 1.0)),
            "tdc": float(qcfg.get("thresholds", {}).get("d_cov", 0.02)),
            "max_wait_ms": float(qcfg.get("max_wait_ms", 20)),
        }
        
        # 使用GPU（threading可以安全使用GPU）
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        print(f"[Quality] Using device: {self.device}")
        
        # 启动worker线程
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()
        print(f"[Quality] Worker thread started")

    def submit(self, job):
        """提交质量计算任务，自动注入历史EWMA"""
        try:
            # 注入历史EWMA状态
            kf_id = job.get("kf_id")
            if kf_id is not None and kf_id in self.ewma_state:
                job["cov_ewma"] = self.ewma_state[kf_id]
            
            self.job_q.put_nowait(job)
        except queue.Full:
            print(f"[Quality] Job queue full, dropping job for kf_id {job.get('kf_id')}")

    def register_callback(self, callback):
        """注册结果通知回调"""
        with self.callback_lock:
            self.callbacks.append(callback)

    def poll(self):
        """轮询结果队列"""
        n = 0
        while True:
            try:
                msg = self.res_q.get_nowait()
            except queue.Empty:
                break
            
            self._process_result(msg)
            n += 1
        return n

    def _process_result(self, msg):
        """处理单个结果：存储、更新统计、通知回调"""
        kf_id = msg.get("kf_id")
        frame_id = msg.get("frame_id")
        
        # 双索引存储
        if kf_id is not None:
            self.cache_by_kf_id[kf_id] = msg
        if frame_id is not None:
            self.cache_by_frame_id[frame_id] = msg
        
        # 保存EWMA状态（NumPy格式）
        if kf_id is not None and "cov_ewma" in msg:
            self.ewma_state[kf_id] = msg["cov_ewma"]
        
        # 更新全局统计
        self._update_global_stats(msg)
        
        # 通知回调
        with self.callback_lock:
            for cb in self.callbacks:
                try:
                    cb(msg)
                except Exception as e:
                    print(f"[Quality] Callback error: {e}")

    def _update_global_stats(self, result):
        """更新全局统计量（滑动窗口）"""
        with self.stats_lock:
            self.stats_window.append({
                "r": result.get("r"),
                "u": result.get("u")
            })
            
            # 每10个结果重新计算统计量
            if len(self.stats_window) >= 10:
                try:
                    all_r = []
                    all_u = []
                    for x in self.stats_window:
                        if x["r"] is not None:
                            r_flat = x["r"].flatten() if hasattr(x["r"], 'flatten') else np.array(x["r"]).flatten()
                            all_r.extend(r_flat)
                        if x["u"] is not None:
                            u_flat = x["u"].flatten() if hasattr(x["u"], 'flatten') else np.array(x["u"]).flatten()
                            all_u.extend(u_flat)
                    
                    if all_r:
                        all_r = torch.as_tensor(all_r, dtype=torch.float32)
                        self.global_stats["r_median"] = float(torch.median(all_r))
                        self.global_stats["r_mad"] = float(torch.median(torch.abs(all_r - self.global_stats["r_median"])))
                    
                    if all_u:
                        all_u = torch.as_tensor(all_u, dtype=torch.float32)
                        self.global_stats["u_median"] = float(torch.median(all_u))
                        self.global_stats["u_mad"] = float(torch.median(torch.abs(all_u - self.global_stats["u_median"])))
                except Exception as e:
                    print(f"[Quality] Stats update error: {e}")

    def get(self, kf_id):
        """按kf_id查询（向后兼容）"""
        self.poll()
        return self.cache_by_kf_id.get(int(kf_id), None)

    def get_by_kf_id(self, kf_id):
        """按kf_id查询"""
        self.poll()
        return self.cache_by_kf_id.get(int(kf_id), None)

    def get_by_frame_id(self, frame_id):
        """按frame_id查询"""
        self.poll()
        return self.cache_by_frame_id.get(int(frame_id), None)

    def _worker_loop(self):
        """Worker线程主循环"""
        print(f"[Quality] Worker loop started on device {self.device}")
        torch.set_grad_enabled(False)
        
        while not self.stop_event.is_set():
            try:
                # 收集批次
                jobs = self._collect_batch()
                if not jobs:
                    continue
                
                # 批量计算
                results = compute_batch(
                    jobs,
                    ps=self.cfg["patch_size"],
                    alpha=self.cfg["alpha"],
                    b0=self.cfg["b0"],
                    theta0=self.cfg["theta0"],
                    C_thr=self.cfg["C_thr"],
                    Q_thr=self.cfg["Q_thr"],
                    thr_zr=self.cfg["tzr"],
                    thr_zu=self.cfg["tzu"],
                    thr_dc=self.cfg["tdc"],
                    device=self.device
                )
                
                # 返回结果
                for r in results:
                    try:
                        self.res_q.put_nowait(r)
                    except queue.Full:
                        print(f"[Quality] Result queue full, dropping result for kf_id {r.get('kf_id')}")
                        
            except Exception as e:
                print(f"[Quality] Worker error: {e}")
                import traceback
                traceback.print_exc()
        
        print(f"[Quality] Worker loop terminated")

    def _collect_batch(self):
        """智能批次收集"""
        jobs = []
        
        # 阻塞等待第一个任务
        try:
            job = self.job_q.get(timeout=0.1)
            if "__quit__" in job:
                self.stop_event.set()
                return []
            
            # 准备job
            prepared = _prep_job(job, self.device)
            jobs.append(prepared)
        except queue.Empty:
            return []
        
        # 尝试收集更多（带超时）
        deadline = time.time() + self.cfg["max_wait_ms"] / 1000.0
        while len(jobs) < self.cfg["batch_size"]:
            remaining = max(0.001, deadline - time.time())
            if remaining <= 0:
                break
            
            try:
                job = self.job_q.get(timeout=remaining)
                if "__quit__" in job:
                    self.stop_event.set()
                    break
                
                prepared = _prep_job(job, self.device)
                jobs.append(prepared)
            except queue.Empty:
                break
        
        return jobs

    def shutdown(self, timeout=1.0):
        """优雅关闭"""
        print(f"[Quality] Shutting down...")
        try:
            self.job_q.put({"__quit__": True}, timeout=0.5)
        except (queue.Full, Exception):
            pass
        
        self.stop_event.set()
        self.worker.join(timeout=timeout)
        
        if self.worker.is_alive():
            print(f"[Quality] Worker thread did not terminate gracefully")
        else:
            print(f"[Quality] Worker thread terminated successfully")