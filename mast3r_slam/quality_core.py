import torch
import torch.nn.functional as F

def _hw(x,h,w):
    if x is None: return None
    if x.ndim==1 or (x.ndim==2 and x.shape[-1]==1): x=x.view(h,w)
    return x

def _grid(x,ps):
    h,w=x.shape[-2:]
    gh,gw=h//ps,w//ps
    x=x[:gh*ps,:gw*ps].view(gh,ps,gw,ps).permute(0,2,1,3).contiguous().view(gh,gw,ps*ps)
    return x,gh,gw

def reduce_grid(x,h,w,ps,valid=None,method="median"):
    x=_hw(x,h,w)
    if valid is not None: valid=_hw(valid,h,w).to(torch.bool)
    X,gh,gw=_grid(x,ps)
    if valid is None:
        if method=="median": v=torch.nanmedian(X,dim=-1).values
        else: v=X.mean(-1)
        return v
    M,_,__=_grid(valid.float(),ps)
    X=X.masked_fill(M<0.5, float("nan"))
    v=torch.nanmedian(X,dim=-1).values if method=="median" else torch.nanmean(X,dim=-1)
    v=torch.nan_to_num(v, nan=0.0)
    return v

def upsample_to_hw(g,h,w,mode="bilinear"):
    g=g.unsqueeze(0).unsqueeze(0)
    y=F.interpolate(g,(h,w),mode=mode,align_corners=False if mode!="nearest" else None)
    return y[0,0]

def view_weight(t_norm,theta,b0,theta0,device):
    t=torch.clamp(t_norm/b0,0,1) if b0>0 else torch.ones((),device=device)
    r=torch.clamp(theta/theta0,0,1) if theta0>0 else torch.ones((),device=device)
    return 0.5*(t+r)

def ema_delta(prev,inc,alpha):
    new=alpha*prev+(1-alpha)*inc
    return new,new-prev

def u_from_CQ(C,Q,C_thr,Q_thr,h,w,ps,quant=0.5):
    C=_hw(C,h,w); Q=_hw(Q,h,w)
    Cn=torch.clamp(C/(C_thr+1e-8),0,1)
    Qn=torch.clamp(Q/(Q_thr+1e-8),0,1)
    U=1-torch.sqrt(torch.clamp(Cn*Qn,0,1))
    g=reduce_grid(U,h,w,ps,method="median")
    return g

def r_from_scalar(r,h,w,ps,valid=None):
    return reduce_grid(r,h,w,ps,valid=valid,method="median")

def valid_grid(valid,h,w,ps):
    v=reduce_grid(valid.float(),h,w,ps,method="mean")
    return (v>0).float()

def robust_z(x,eps=1e-6):
    m=torch.median(x)
    mad=torch.median(torch.abs(x-m))+eps
    return (x-m)/mad

def classify(delta_cov, r, u, thr_zr=1.0, thr_zu=1.0, thr_dc=0.02):
    # Store original shape for later reshape
    original_shape = delta_cov.shape
    
    # Flatten all inputs to ensure consistent 1D operations
    delta_cov = delta_cov.flatten()
    r = r.flatten()
    u = u.flatten()
    
    # Compute robust z-scores
    zr = robust_z(r)
    zu = robust_z(u)
    
    # Classification conditions
    c1 = (delta_cov < thr_dc) & (zu > thr_zu)
    c2 = (delta_cov >= thr_dc) & (zr > thr_zr) & (zu > thr_zu)
    c3 = (zr > thr_zr) & (zu <= thr_zu)
    
    # Initialize class labels
    cls = torch.zeros_like(r, dtype=torch.long)
    cls[c1] = 1
    cls[c2] = 2
    cls[c3] = 3
    
    # Initialize priority scores
    p = torch.zeros_like(r)
    
    # Compute priority scores with proper masking
    mask1 = (cls == 1)
    mask2 = (cls == 2)
    mask3 = (cls == 3)
    
    if mask1.any():
        p[mask1] = ((1 - torch.clamp(delta_cov, 0, 1)) + torch.clamp(zu, 0))[mask1]
    if mask2.any():
        p[mask2] = (torch.clamp(zr, 0) + torch.clamp(zu, 0))[mask2]
    if mask3.any():
        p[mask3] = (torch.clamp(zr, 0) + torch.clamp(1 - u, 0))[mask3]
    
    # Normalize priority scores
    p = p / (p.max() + 1e-6)
    
    # Reshape back to original shape
    cls = cls.reshape(original_shape)
    p = p.reshape(original_shape)
    
    return cls, p

def pack_result(kfid, dc, r, u, cls, pri, ewma):
    def to_numpy(x):
        if torch.is_tensor(x):
            return x.cpu().numpy()
        return x
    
    return {
        "kf_id": int(kfid),
        "delta_cov": to_numpy(dc),
        "r": to_numpy(r),
        "u": to_numpy(u),
        "class_id": to_numpy(cls),
        "priority": to_numpy(pri),
        "cov_ewma": to_numpy(ewma)
    }

def compute_batch(batch,ps,alpha,b0,theta0,C_thr,Q_thr,thr_zr,thr_zu,thr_dc,device):
    outs=[]
    for jb in batch:
        h,w=jb["H"],jb["W"]
        valid=_hw(jb["valid_kf"].to(device),h,w)
        inc=valid_grid(valid,h,w,ps)*view_weight(jb["t_norm"].to(device),jb["theta"].to(device),b0,theta0,device)
        prev=jb.get("cov_ewma",None)
        if prev is None: prev=torch.zeros_like(inc,device=device)
        ew,dc=ema_delta(prev,inc,alpha)
        r=r_from_scalar(_hw(jb["r_pix"].to(device),h,w),h,w,ps,valid=valid)
        u=u_from_CQ(jb["Ck"].to(device),jb["Qk"].to(device),C_thr,Q_thr,h,w,ps)
        cls,pri=classify(dc,r,u,thr_zr,thr_zu,thr_dc)
        outs.append(pack_result(jb["kf_id"],dc,r,u,cls,pri,ew))
    return outs