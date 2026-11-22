import time

import cv2
import numpy as np

COARSE_W = 640
PROC_W = 1280
MAX_TILT_DEG = 25
INK_THRESHOLDS = (0.20, 0.12)
MAX_CAND = 12
EDGE_MIN_SUPPORT = 0.5
PART_MIN_SUPPORT = 0.55
ALL_MIN_SUPPORT = 0.4
DIVIDER_MIN_SUPPORT = 0.6
MIN_PAPER_FRAC = 0.35
DIVIDER_MAX_DEV_DEG = 6


def _odd(x):
    x = int(round(x))
    return x if x % 2 else x + 1


def _to_bgr(img):
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if img.shape[2] == 4:
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img


def _median_frame(frames):
    if isinstance(frames, np.ndarray):
        return _to_bgr(frames)
    frames = [_to_bgr(f) for f in frames if f is not None]
    if not frames:
        return None
    shape = frames[0].shape
    frames = [f for f in frames if f.shape == shape]
    if len(frames) == 1:
        return frames[0]
    return np.median(np.stack(frames), axis=0).astype(np.uint8)


def _resize_w(img, w):
    s = w / img.shape[1]
    if abs(s - 1) < 1e-6:
        return img, 1.0
    return cv2.resize(img, (w, int(round(img.shape[0] * s))), interpolation=cv2.INTER_AREA), s


def _scale_pts(p, s_from, s_to):
    p = np.asarray(p, np.float64)
    return (p + 0.5) / s_from * s_to - 0.5


def noise_level(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    d = np.abs(g.astype(np.int16) - cv2.medianBlur(g, 3))
    return 1.4826 * float(np.mean(d)) / 1.25 if d.size else 0.0


def ink_contrast(img, k, smooth=0.0):
    f = cv2.GaussianBlur(img, (0, 0), smooth) if smooth > 0.3 else img
    clo = cv2.morphologyEx(f, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
    clo_f = clo.astype(np.float32)
    ref = np.percentile(clo_f[::4, ::4].reshape(-1, clo_f.shape[2]), 95, axis=0)
    ratio = (clo_f - f) / np.maximum(clo_f, np.maximum(25.0, 0.45 * ref))
    return np.clip(ratio.min(axis=2), 0, 1)


def paper_fraction(img, quad):
    h, w = img.shape[:2]
    m = np.zeros((h, w), np.uint8)
    cv2.fillPoly(m, [np.rint(quad).astype(np.int32)], 1)
    px = img[m > 0].astype(np.float32)
    if len(px) < 50:
        return 0.0
    wp = np.percentile(px, 85, axis=0) + 1.0
    n = px / wp
    chroma = n.max(1) - n.min(1)
    bright = n.mean(1)
    return float(((chroma < 0.12) & (bright > 0.45)).mean())


def _line_from_hough(rho, th, vertical):
    n = np.array([np.cos(th), np.sin(th)])
    c = rho
    if (vertical and n[0] < 0) or (not vertical and n[1] < 0):
        n, c = -n, -c
    return n, c


def _intersect_nc(n1, c1, n2, c2):
    det = n1[..., 0] * n2[..., 1] - n1[..., 1] * n2[..., 0]
    det = np.where(np.abs(det) < 1e-9, 1e-9, det)
    x = (c1 * n2[..., 1] - c2 * n1[..., 1]) / det
    y = (n1[..., 0] * c2 - n2[..., 0] * c1) / det
    return np.stack([x, y], -1)


def _isect_pd(l1, l2):
    (p, r), (q, s) = l1, l2
    den = r[0] * s[1] - r[1] * s[0]
    if abs(den) < 1e-9:
        return None
    t = ((q[0] - p[0]) * s[1] - (q[1] - p[1]) * s[0]) / den
    return p + t * r


def _candidates(mask, vertical, votes_min):
    h, w = mask.shape
    tilt = np.radians(MAX_TILT_DEG)
    if vertical:
        ranges = [(0.0, tilt), (np.pi - tilt, np.pi)]
    else:
        ranges = [(np.pi / 2 - tilt, np.pi / 2 + tilt)]
    found = []
    for lo, hi in ranges:
        L = cv2.HoughLinesWithAccumulator(mask, 1, np.pi / 360, votes_min, min_theta=lo, max_theta=hi)
        if L is not None:
            found += [tuple(x) for x in L[:, 0, :]]
    found.sort(key=lambda r: -r[2])
    refs = np.array([0.2, 0.8]) * (h if vertical else w)
    tol = max(4.0, 0.012 * w)
    keep = []
    for rho, th, votes in found:
        n, c = _line_from_hough(rho, th, vertical)
        pos = (c - n[1] * refs) / n[0] if vertical else (c - n[0] * refs) / n[1]
        if any(np.all(np.abs(pos - k[3]) < tol) for k in keep):
            continue
        keep.append((n, c, votes, pos))
        if len(keep) >= MAX_CAND:
            break
    return keep


_T = np.linspace(0.03, 0.97, 48)
_TW = np.where((_T < 0.15) | (_T > 0.85), 2.0, 1.0)
_TW /= _TW.sum()


def _seg_support(sup_map, A, B):
    h, w = sup_map.shape
    P = A[..., None, :] + _T[:, None] * (B - A)[..., None, :]
    x = np.rint(P[..., 0]).astype(np.int64)
    y = np.rint(P[..., 1]).astype(np.int64)
    inside = (x >= 0) & (x < w) & (y >= 0) & (y < h)
    vals = sup_map[np.clip(y, 0, h - 1), np.clip(x, 0, w - 1)] * inside
    return (vals * _TW).sum(-1), np.linalg.norm(B - A, axis=-1)


def _structure(mask, sup_map):
    h, w = mask.shape
    votes_min = max(15, int(0.08 * min(h, w)))
    V = _candidates(mask, True, votes_min)
    Hc = _candidates(mask, False, votes_min)
    if len(V) < 3 or len(Hc) < 2:
        return None, {"n_vert": len(V), "n_hori": len(Hc)}
    nv, nh = len(V), len(Hc)
    Vn = np.array([v[0] for v in V]); Vc = np.array([v[1] for v in V])
    Hn = np.array([x[0] for x in Hc]); Hcc = np.array([x[1] for x in Hc])
    P = _intersect_nc(Vn[:, None, :], Vc[:, None], Hn[None, :, :], Hcc[None, :])
    SV, LV = _seg_support(sup_map, P[:, :, None, :], P[:, None, :, :])
    PH = P.transpose(1, 0, 2)
    SH, LH = _seg_support(sup_map, PH[:, :, None, :], PH[:, None, :, :])

    t, b, l, m, r = (a.ravel() for a in np.meshgrid(np.arange(nh), np.arange(nh), np.arange(nv),
                                                     np.arange(nv), np.arange(nv), indexing="ij"))
    ok = (t != b) & (l != m) & (m != r) & (l != r)
    t, b, l, m, r = t[ok], b[ok], l[ok], m[ok], r[ok]
    TL, TM, TR = P[l, t], P[m, t], P[r, t]
    BL, BM, BR = P[l, b], P[m, b], P[r, b]
    W_t, W_b = TR[:, 0] - TL[:, 0], BR[:, 0] - BL[:, 0]
    ok = (W_t > 0.12 * w) & (W_b > 0.12 * w)
    for (A, Bq, W_) in ((TL, TM, W_t), (TM, TR, W_t), (BL, BM, W_b), (BM, BR, W_b)):
        ok &= (Bq[:, 0] - A[:, 0]) > 0.15 * np.maximum(W_, 1)
    for (A, Bq) in ((TL, BL), (TM, BM), (TR, BR)):
        ok &= (Bq[:, 1] - A[:, 1]) > 0.12 * h
    def ang(A, Bq):
        return np.degrees(np.arctan2(Bq[:, 0] - A[:, 0], Bq[:, 1] - A[:, 1]))
    aL, aM, aR = ang(TL, BL), ang(TM, BM), ang(TR, BR)
    frac = ((TM[:, 0] + BM[:, 0]) - (TL[:, 0] + BL[:, 0])) / np.maximum((TR[:, 0] + BR[:, 0]) - (TL[:, 0] + BL[:, 0]), 1)
    ok &= np.abs(aM - ((1 - frac) * aL + frac * aR)) < DIVIDER_MAX_DEV_DEG
    allc = np.stack([TL, TM, TR, BL, BM, BR], 1)
    ok &= np.all((allc[..., 0] > -0.02 * w) & (allc[..., 0] < 1.02 * w) &
                 (allc[..., 1] > -0.02 * h) & (allc[..., 1] < 1.02 * h), axis=1)
    if not ok.any():
        return None, {"n_vert": nv, "n_hori": nh}
    t, b, l, m, r = t[ok], b[ok], l[ok], m[ok], r[ok]
    sup = np.stack([SV[l, t, b], SV[m, t, b], SV[r, t, b], SH[t, l, m], SH[t, m, r], SH[b, l, m], SH[b, m, r]], 1)
    ln = np.stack([LV[l, t, b], LV[m, t, b], LV[r, t, b], LH[t, l, m], LH[t, m, r], LH[b, l, m], LH[b, m, r]], 1)
    good = sup.min(1) >= EDGE_MIN_SUPPORT
    if not good.any():
        return None, {"n_vert": nv, "n_hori": nh, "best_min_support": round(float(sup.min(1).max()), 3)}
    score = (ln * (2 * sup - 1)).sum(1)
    score[~good] = -np.inf
    i = int(np.argmax(score))
    lines = {"T": (Hn[t[i]], Hcc[t[i]]), "B": (Hn[b[i]], Hcc[b[i]]),
             "L": (Vn[l[i]], Vc[l[i]]), "M": (Vn[m[i]], Vc[m[i]]), "R": (Vn[r[i]], Vc[r[i]])}
    names = ["L", "M", "R", "T1", "T2", "B1", "B2"]
    info = {"n_vert": nv, "n_hori": nh, "score": round(float(score[i]), 1),
            "coarse_support": {k: round(float(s), 3) for k, s in zip(names, sup[i])}}
    return lines, info


class _Refiner:
    def __init__(self, img):
        self.h, self.w = img.shape[:2]
        sm = float(np.clip(0.12 * noise_level(img) - 0.2, 0, 1.6))
        con = ink_contrast(img, _odd(max(7, 0.015 * self.w)), sm)
        self.wmap = np.clip(con - 0.08, 0, None).astype(np.float32)
        thin = ink_contrast(img, _odd(max(5, 0.0085 * self.w)), sm)
        self.tmap = np.clip(thin - 0.08, 0, None).astype(np.float32)
        self.band = max(8, int(round(0.014 * self.w)))
        self.nband = max(4, int(round(0.005 * self.w)))
        self.ex = max(5, int(round(0.0055 * self.w)))

    def centres(self, p0, p1, band, vertical):
        p0, p1 = np.asarray(p0, np.float64), np.asarray(p1, np.float64)
        ax = 1 if vertical else 0
        if abs(p1[ax] - p0[ax]) < 1e-6:
            return np.zeros((0, 2)), np.zeros(0), 0
        lim = self.h if vertical else self.w
        s = np.arange(int(np.ceil(min(p0[ax], p1[ax]))), int(np.floor(max(p0[ax], p1[ax]))) + 1)
        n_exp = len(s)
        s = s[(s >= 0) & (s < lim)]
        if len(s) == 0:
            return np.zeros((0, 2)), np.zeros(0), n_exp
        pred = p0[1 - ax] + (p1[1 - ax] - p0[1 - ax]) * (s - p0[ax]) / (p1[ax] - p0[ax])
        off = np.arange(-band, band + 1)
        q = np.rint(pred).astype(np.int64)[:, None] + off[None, :]
        lim2 = self.w if vertical else self.h
        valid = (q >= 0) & (q < lim2)
        qc = np.clip(q, 0, lim2 - 1)
        wv = (self.wmap[s[:, None], qc] if vertical else self.wmap[qc, s[:, None]]) * valid
        tot = wv.sum(1)
        keep = tot > 0.3
        cen = (wv * q).sum(1)[keep] / tot[keep]
        s = s[keep]
        pts = np.stack([cen, s], 1) if vertical else np.stack([s, cen], 1)
        return pts.astype(np.float64), np.minimum(tot[keep], 2.5), n_exp

    @staticmethod
    def robust_fit(pts, w0, iters=8):
        if len(pts) < 6:
            return None
        w = w0.astype(np.float64).copy()
        mu = d = None
        for _ in range(iters):
            if w.sum() < 1e-6:
                return None
            mu = (w[:, None] * pts).sum(0) / w.sum()
            X = pts - mu
            C = (w[:, None, None] * X[:, :, None] * X[:, None, :]).sum(0)
            _, ev = np.linalg.eigh(C)
            d = ev[:, 1]
            n = np.array([-d[1], d[0]])
            res = X @ n
            sig = max(0.5, 1.4826 * np.median(np.abs(res)))
            u = res / (4.0 * sig)
            w = w0 * np.where(np.abs(u) < 1, (1 - u * u) ** 2, 0.0)
        return mu, d

    def radon(self, c, u, s_lo, s_hi, band, max_ang, D=None):
        s = np.arange(s_lo, s_hi + 1e-6, 1.0)
        s = s[np.abs(s) >= self.ex]
        if len(s) < 8:
            return None
        if D is None:
            ws = np.ones(len(s))
        else:
            ws = 1.0 / (1.0 + (np.abs(s) / D) ** 2) + 2.0 * (np.abs(s) <= max(2 * self.ex, 0.35 * D))
        n = np.array([-u[1], u[0]])
        ang = np.radians(np.arange(-max_ang, max_ang + 1e-6, 0.5))
        off = np.arange(-band, band + 1e-6, 0.5)
        ua = np.cos(ang)[:, None] * u + np.sin(ang)[:, None] * n
        na = -np.sin(ang)[:, None] * u + np.cos(ang)[:, None] * n
        P = (c + off[None, :, None, None] * na[:, None, None, :] + s[None, None, :, None] * ua[:, None, None, :])
        P = P.reshape(-1, len(s), 2).astype(np.float32)
        vals = cv2.remap(self.tmap, np.ascontiguousarray(P[..., 0]), np.ascontiguousarray(P[..., 1]),
                         cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        sc = (np.minimum(vals, 0.25) * ws.astype(np.float32)).sum(1).reshape(len(ang), len(off))
        sc = cv2.GaussianBlur(sc.astype(np.float32), (5, 1), 0, borderType=cv2.BORDER_REPLICATE) \
            if len(off) >= 5 else sc
        ia, io = np.unravel_index(int(np.argmax(sc)), sc.shape)
        return c + off[io] * na[ia], ua[ia], float(sc[ia, io]) / (0.25 * ws.sum())

    def fit_window(self, c, u, s_lo, s_hi, vertical, band, max_ang, D=None):
        rr = self.radon(c, u, s_lo, s_hi, band, max_ang, D)
        if rr is None:
            return None
        p, d, _ = rr
        a0, a1 = c + s_lo * u, c + s_hi * u
        q0 = p + d * float((a0 - p) @ d)
        q1 = p + d * float((a1 - p) @ d)
        pts, wt, n_exp = self.centres(q0, q1, self.nband, vertical)
        if len(pts):
            sd = np.abs((pts - c) @ u)
            keep = sd >= self.ex
            n_exp = max(1, n_exp - int(np.sum(np.abs(np.arange(s_lo, s_hi + 1)) < self.ex)))
            pts, wt, sd = pts[keep], wt[keep], sd[keep]
            if D is not None:
                wt = wt / (1.0 + (sd / D) ** 2)
        fit = self.robust_fit(pts, wt)
        if fit is None:
            return None
        return fit, len(pts) / max(n_exp, 1)

    def global_line(self, A0, A1, vertical, cuts):
        A0, A1 = np.asarray(A0, np.float64), np.asarray(A1, np.float64)
        L = np.linalg.norm(A1 - A0)
        u = (A1 - A0) / L
        c = (A0 + A1) / 2
        r = self.fit_window(c, u, -L / 2 + self.ex, L / 2 - self.ex, vertical, self.band, 3.0)
        return None if r is None else r[0]

    def local_line(self, line, A0, A1, corner, vertical, D):
        A0, A1, corner = (np.asarray(x, np.float64) for x in (A0, A1, corner))
        p, d = line
        if (A1 - A0) @ d < 0:
            d = -d
        c = p + d * float((corner - p) @ d)
        t0, t1 = float((A0 - c) @ d), float((A1 - c) @ d)
        Lfull = t1 - t0
        best = None
        for Dk in (D, 2 * D, 0.45 * Lfull):
            s_lo, s_hi = max(t0, -Dk), min(t1, Dk)
            r = self.fit_window(c, d, s_lo, s_hi, vertical, self.band, 4.0, D)
            if r is not None and (best is None or r[1] > best[1] + 0.15):
                best = r
            if best is not None and best[1] >= 0.5:
                break
        return None if best is None else best[0]

    def peaks(self, p0, p1, vertical, band):
        p0, p1 = np.asarray(p0, np.float64), np.asarray(p1, np.float64)
        L = np.linalg.norm(p1 - p0)
        m = min(0.2, (self.ex + 4) / max(L, 1))
        a, b = p0 + m * (p1 - p0), p1 - m * (p1 - p0)
        ax = 1 if vertical else 0
        if abs(b[ax] - a[ax]) < 1:
            return np.zeros(0)
        lim, lim2 = (self.h, self.w) if vertical else (self.w, self.h)
        s = np.arange(int(np.ceil(min(a[ax], b[ax]))), int(np.floor(max(a[ax], b[ax]))) + 1)
        pred = a[1 - ax] + (b[1 - ax] - a[1 - ax]) * (s - a[ax]) / (b[ax] - a[ax])
        q = np.rint(pred).astype(np.int64)[:, None] + np.arange(-band, band + 1)[None, :]
        valid = (q >= 0) & (q < lim2) & (s[:, None] >= 0) & (s[:, None] < lim)
        sc = np.clip(s, 0, lim - 1)[:, None]
        qc = np.clip(q, 0, lim2 - 1)
        v = (self.wmap[sc, qc] if vertical else self.wmap[qc, sc]) * valid
        return v.max(1)


def _quad_ok(q, w, h):
    q = np.asarray(q, np.float32)
    if not cv2.isContourConvex(q.reshape(-1, 1, 2)):
        return False
    if cv2.contourArea(q) < 0.01 * w * h:
        return False
    for i in range(4):
        a, b, c = q[i - 1], q[i], q[(i + 1) % 4]
        v1, v2 = a - b, c - b
        cosang = float(v1 @ v2 / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-9))
        if abs(cosang) > np.cos(np.radians(45)):
            return False
    return True


CORNERS = {"TL": ("T", "L"), "TM": ("T", "M"), "TR": ("T", "R"), "BL": ("B", "L"), "BM": ("B", "M"), "BR": ("B", "R")}
SEGS = {"T": ("TL", "TR", False), "B": ("BL", "BR", False), "L": ("TL", "BL", True), "M": ("TM", "BM", True),
        "R": ("TR", "BR", True)}


def _detect_lr(img_full, part, dbg):
    t0 = time.perf_counter()
    H0, W0 = img_full.shape[:2]
    proc, s_proc = _resize_w(img_full, min(PROC_W, W0))
    coarse, s_c = _resize_w(img_full, min(COARSE_W, W0))
    hc, wc = coarse.shape[:2]
    nl = noise_level(coarse)
    sm = float(np.clip(0.15 * nl - 0.3, 0, 1.5))
    dbg["noise"] = round(nl, 2)
    con_c = ink_contrast(coarse, _odd(max(7, 0.015 * wc)), sm)
    lines = None
    for thr in INK_THRESHOLDS:
        mask = (con_c > thr).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8)) if sm > 0 else mask
        dist = cv2.distanceTransform(255 - mask, cv2.DIST_L2, 3)
        sup_map = np.clip((6.0 - dist) / 3.0, 0, 1).astype(np.float32)
        lines, info = _structure(mask, sup_map)
        dbg.setdefault("coarse_tries", []).append({"thr": thr, **info})
        if lines is not None:
            dbg["ink_thr"] = thr
            break
    dbg["t_coarse"] = round(time.perf_counter() - t0, 4)
    if lines is None:
        dbg["fail"] = "no border+divider structure"
        return None

    R = _Refiner(proc)
    cur = {}
    for k, (n, c) in lines.items():
        d = np.array([-n[1], n[0]])
        p0 = n * c
        cur[k] = (_scale_pts(p0, s_c, s_proc), d)
    corners = {k: _isect_pd(cur[a], cur[b]) for k, (a, b) in CORNERS.items()}
    dbg["coarse_corners"] = {k: np.round(_scale_pts(p, s_proc, 1.0), 1).tolist() for k, p in corners.items()}
    for k, (e0, e1, vert) in SEGS.items():
        g = R.global_line(corners[e0], corners[e1], vert, None)
        if g is not None:
            cur[k] = g
        else:
            dbg.setdefault("global_fail", []).append(k)
    gcorners = {k: _isect_pd(cur[a], cur[b]) for k, (a, b) in CORNERS.items()}
    if any(p is None or np.linalg.norm(p - corners[k]) > 0.04 * R.w for k, p in gcorners.items()):
        dbg["fail"] = "refinement diverged"
        return None
    corners = gcorners
    paper_w = np.linalg.norm(corners["TR"] - corners["TL"])
    paper_h = np.linalg.norm(corners["BL"] - corners["TL"])
    for it in range(2):
        new = {}
        for k, (a, b) in CORNERS.items():
            fits = []
            for ln in (a, b):
                e0, e1, vert = SEGS[ln]
                D = float(np.clip(0.22 * (paper_h if vert else paper_w), 25, 0.12 * R.w))
                fits.append(R.local_line(cur[ln], corners[e0], corners[e1], corners[k], vert, D))
            p = None if any(f is None for f in fits) else _isect_pd(fits[0], fits[1])
            if p is None or np.linalg.norm(p - gcorners[k]) > max(10.0, 0.02 * R.w):
                new[k] = corners[k]
                dbg.setdefault("local_reject", []).append(k)
            else:
                new[k] = p
        corners = new
    dbg["t_refine"] = round(time.perf_counter() - t0, 4)

    edges = {"L": ("TL", "BL", True), "M": ("TM", "BM", True), "R": ("TR", "BR", True), "T1": ("TL", "TM", False),
             "T2": ("TM", "TR", False), "B1": ("BL", "BM", False), "B2": ("BM", "BR", False)}
    pk = {e: R.peaks(corners[a], corners[b], v, R.nband + 5) for e, (a, b, v) in edges.items()}
    allpk = np.concatenate([x for x in pk.values() if len(x)]) if any(len(x) for x in pk.values()) else np.zeros(1)
    ref = float(np.percentile(allpk, 75))
    thr_pk = max(0.04, 0.25 * ref)
    sup = {e: float((x >= thr_pk).mean()) if len(x) else 0.0 for e, x in pk.items()}
    dbg["marker_contrast"] = round(ref + 0.08, 3)
    if ref + 0.08 < 0.12:
        dbg["fail"] = "lines too faint"
        return None
    dbg["support"] = {k: round(v, 3) for k, v in sup.items()}
    full = {k: _scale_pts(p, s_proc, 1.0).astype(np.float32) for k, p in corners.items()}
    q_left = np.array([full["TL"], full["TM"], full["BM"], full["BL"]], np.float32)
    q_right = np.array([full["TM"], full["TR"], full["BR"], full["BM"]], np.float32)
    q_outer = np.array([full["TL"], full["TR"], full["BR"], full["BL"]], np.float32)
    part_edges = ("L", "T1", "M", "B1") if part == "left" else ("M", "T2", "R", "B2")
    part_sup = min(sup[e] for e in part_edges)
    all_sup = float(np.mean(list(sup.values())))
    if not all(_quad_ok(q, W0, H0) for q in (q_left, q_right, q_outer)):
        dbg["fail"] = "bad geometry"
        return None
    def _ang(a, b):
        return np.degrees(np.arctan2(b[0] - a[0], b[1] - a[1]))
    aL, aM, aR = _ang(full["TL"], full["BL"]), _ang(full["TM"], full["BM"]), _ang(full["TR"], full["BR"])
    xs = [(full[a][0] + full[b][0]) / 2 for a, b in (("TL", "BL"), ("TM", "BM"), ("TR", "BR"))]
    frac = (xs[1] - xs[0]) / max(xs[2] - xs[0], 1e-6)
    dev = abs(aM - ((1 - frac) * aL + frac * aR))
    dbg["divider_dev_deg"] = round(float(dev), 2)
    if dev > DIVIDER_MAX_DEV_DEG or not (0.15 < frac < 0.85):
        dbg["fail"] = f"divider not parallel to the borders ({dev:.1f} deg)"
        return None
    paper_frac = paper_fraction(coarse, _scale_pts(q_outer, 1.0, s_c))
    dbg["paper_frac"] = round(paper_frac, 3)
    if part_sup < PART_MIN_SUPPORT or min(sup.values()) < ALL_MIN_SUPPORT or sup["M"] < DIVIDER_MIN_SUPPORT:
        dbg["fail"] = f"weak lines (part support {part_sup:.2f})"
        return None
    if paper_frac < MIN_PAPER_FRAC:
        dbg["fail"] = f"inside is not paper ({paper_frac:.2f})"
        return None
    conf = float(np.clip(part_sup, 0, 1) * np.clip(all_sup, 0, 1))
    return {"quad": q_left if part == "left" else q_right,
            "parts": {"left": q_left, "right": q_right},
            "paper_quad": q_outer,
            "middle": np.array([full["TM"], full["BM"]], np.float32),
            "corners": full,
            "confidence": round(conf, 3)}


def _order_quad(q):
    q = np.asarray(q, np.float32)
    c = q.mean(0)
    ang = np.arctan2(q[:, 1] - c[1], q[:, 0] - c[0])
    q = q[np.argsort(ang)]
    i = int(np.argmin(q.sum(1)))
    return np.roll(q, -i, axis=0)


def find_work_area(frames, part="left", debug=False):
    t0 = time.perf_counter()
    if part not in ("left", "right", "top", "bottom"):
        raise ValueError(f"part must be left/right/top/bottom, not {part!r}")
    img = _median_frame(frames)
    if img is None:
        return None
    dbg = {}
    if part in ("left", "right"):
        res = _detect_lr(img, part, dbg)
    else:
        imgT = np.ascontiguousarray(img.transpose(1, 0, 2))
        res = _detect_lr(imgT, "left" if part == "top" else "right", dbg)
        if res is not None:
            def sw(q):
                return _order_quad(np.asarray(q)[:, ::-1])
            res = {"quad": sw(res["quad"]),
                   "parts": {"top": sw(res["parts"]["left"]), "bottom": sw(res["parts"]["right"])},
                   "paper_quad": sw(res["paper_quad"]),
                   "middle": res["middle"][:, ::-1].copy(),
                   "corners": {k: v[::-1].copy() for k, v in res["corners"].items()},
                   "confidence": res["confidence"]}
    dbg["t_total"] = round(time.perf_counter() - t0, 4)
    if res is not None:
        res["part"] = part
        if debug:
            res["debug"] = dbg
        return res
    if debug:
        print("find_work_area: not found:", dbg.get("fail"), dbg.get("coarse_tries"))
    return None


def draw_result(frame, res, color=(0, 220, 0)):
    vis = frame.copy()
    if res is None:
        cv2.putText(vis, "work area NOT found", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        return vis
    cv2.polylines(vis, [np.rint(res["paper_quad"]).astype(np.int32)], True, (160, 160, 160), 1, cv2.LINE_AA)
    a, b = np.rint(res["middle"]).astype(int)
    cv2.line(vis, tuple(map(int, a)), tuple(map(int, b)), (255, 120, 0), 1, cv2.LINE_AA)
    q = res["quad"]
    cv2.polylines(vis, [np.rint(q).astype(np.int32)], True, color, 2, cv2.LINE_AA)
    for i, (x, y) in enumerate(q):
        cv2.circle(vis, (int(round(x)), int(round(y))), 4, (0, 0, 255), -1)
        cv2.putText(vis, str(i + 1), (int(x) + 6, int(y) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
    cv2.putText(vis, f"{res['part']} part, confidence {res['confidence']:.2f}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    return vis


if __name__ == "__main__":
    import sys
    f = cv2.imread(sys.argv[1])
    part = sys.argv[2] if len(sys.argv) > 2 else "left"
    r = find_work_area(f, part, debug=True)
    if r is None:
        print("not found")
    else:
        print(part, np.round(r["quad"], 1).tolist(), "conf", r["confidence"])
        print(r["debug"])
        if len(sys.argv) > 3:
            cv2.imwrite(sys.argv[3], draw_result(f, r))
