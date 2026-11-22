import cv2
import numpy as np

P = dict(
    work_size=360,
    margin=0.07,
    close_k=0.08,
    soft_fac=1.5,
    sharp_seed=0.030,
    sharp_cand=0.012,
    dark_seed=0.40,
    dark_cand=0.12,
    dark_edge_r=0.025,
    chroma_seed=0.22,
    chroma_cand=0.10,
    join_k=0.035,
    seam_len=0.12,
    ink_band=0.055,
    ink_len=0.07,
    ink_max_w=0.028,
    inset=0.012,
    min_area=0.0022,
    max_area=0.60,
    min_seed=0.0003,
    min_edge_px=0.003,
    min_thick=0.007,
    canny_sigma=1.5,
    canny_lo=6,
    canny_hi=16,
    canny_gate=0.0015,
    core_k=0.025,
    neck_ratio=0.6,
    trim=0.12,
    trim_max=0.05,
    contain_frac=0.6,
)


def order_corners(q):
    q = np.asarray(q, np.float32).reshape(4, 2)
    s, d = q.sum(1), q[:, 1] - q[:, 0]
    return np.array([q[np.argmin(s)], q[np.argmin(d)], q[np.argmax(s)], q[np.argmax(d)]], np.float32)


def _odd(v):
    v = int(round(v))
    return max(3, v | 1)


def _rect_k(n):
    return cv2.getStructuringElement(cv2.MORPH_RECT, (n, n))


def _geometry(quad, p):
    q = order_corners(quad)
    wq = 0.5 * (np.linalg.norm(q[1] - q[0]) + np.linalg.norm(q[2] - q[3]))
    hq = 0.5 * (np.linalg.norm(q[3] - q[0]) + np.linalg.norm(q[2] - q[1]))
    s = min(p["work_size"] / np.sqrt(max(wq * hq, 1.0)), 2.0)
    Wr, Hr = max(16, int(round(wq * s))), max(16, int(round(hq * s)))
    size = np.sqrt(Wr * Hr)
    m = int(round(p["margin"] * size))
    dst = np.array([(m, m), (m + Wr, m), (m + Wr, m + Hr), (m, m + Hr)], np.float32)
    H = cv2.getPerspectiveTransform(q, dst)
    return q, H, Wr, Hr, m, size


def _gmag(img, s):
    b = cv2.GaussianBlur(img, (0, 0), s)
    gx = cv2.Sobel(b, cv2.CV_32F, 1, 0, ksize=3, scale=0.125)
    gy = cv2.Sobel(b, cv2.CV_32F, 0, 1, ksize=3, scale=0.125)
    return cv2.magnitude(gx, gy)


def _opp(img):
    y = img.mean(2) + 1.0
    return np.stack([(img[..., 2] - img[..., 1]) / y, (img[..., 2] + img[..., 1] - 2 * img[..., 0]) / (2 * y)], -1)


def _cues(R, p, size):
    Rb = cv2.GaussianBlur(R.astype(np.float32), (0, 0), 0.8)
    Y = Rb.mean(2) + 1.0
    k = _rect_k(_odd(p["close_k"] * size))
    Pg = cv2.morphologyEx(Y, cv2.MORPH_CLOSE, k)
    D = np.clip(1.0 - Y / Pg, 0, 1)
    lY = np.log(Y)
    b1 = cv2.GaussianBlur(lY, (0, 0), 1.0)
    gx = cv2.Sobel(b1, cv2.CV_32F, 1, 0, ksize=3, scale=0.125)
    gy = cv2.Sobel(b1, cv2.CV_32F, 0, 1, ksize=3, scale=0.125)
    sharp = cv2.magnitude(gx, gy) - p["soft_fac"] * _gmag(lY, 3.0)
    bc = cv2.GaussianBlur(lY, (0, 0), p["canny_sigma"])
    dx = np.clip(cv2.Sobel(bc, cv2.CV_32F, 1, 0, ksize=3) * 125.0, -32000, 32000).astype(np.int16)
    dy = np.clip(cv2.Sobel(bc, cv2.CV_32F, 0, 1, ksize=3) * 125.0, -32000, 32000).astype(np.int16)
    canny = cv2.Canny(dx, dy, p["canny_lo"], p["canny_hi"], L2gradient=True)
    canny &= ((sharp > p["canny_gate"]).astype(np.uint8) * 255)
    Pc = cv2.morphologyEx(Rb, cv2.MORPH_CLOSE, k)
    o = _opp(Rb)
    dC = o - _opp(Pc)
    trust = np.clip((Y - 25.0) / 40.0, 0, 1)
    C = np.sqrt((dC ** 2).sum(-1)) * trust
    return dict(Rb=Rb, Y=Y, Pg=Pg, D=D, sharp=sharp, C=C, opp=o, canny=canny)


def _ink_mask(D, C, Wr, Hr, m, size, p):
    hgt, wid = D.shape
    dark = ((D > 0.18) & (C < 0.15)).astype(np.uint8) * 255
    band = int(round(p["ink_band"] * size))
    L = _odd(p["ink_len"] * size)
    wmax = _odd(p["ink_max_w"] * size)
    ink = np.zeros_like(dark)
    vert = cv2.getStructuringElement(cv2.MORPH_RECT, (1, L))
    horz = cv2.getStructuringElement(cv2.MORPH_RECT, (L, 1))
    sides = [
        ((slice(0, hgt), slice(max(0, m - band), m + band)), vert),
        ((slice(0, hgt), slice(m + Wr - band, min(wid, m + Wr + band))), vert),
        ((slice(max(0, m - band), m + band), slice(0, wid)), horz),
        ((slice(m + Hr - band, min(hgt, m + Hr + band)), slice(0, wid)), horz),
    ]
    for sl, se in sides:
        sub = dark[sl]
        lines = cv2.morphologyEx(sub, cv2.MORPH_OPEN, se)
        thick = cv2.morphologyEx(sub, cv2.MORPH_OPEN, _rect_k(wmax))
        lines &= ~cv2.dilate(thick, _rect_k(3))
        ink[sl] |= lines
    return cv2.dilate(ink, _rect_k(_odd(0.012 * size)))


def _fill(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = np.zeros_like(mask)
    cv2.drawContours(out, cnts, -1, 255, -1)
    return out


def _split(region, cores_m, seed, min_seed, p):
    nc, cl, cst, _ = cv2.connectedComponentsWithStats(cores_m & region, connectivity=8)
    big = [i for i in range(1, nc) if cv2.countNonZero(((cl == i) & (seed > 0)).astype(np.uint8)) >= min_seed]
    if len(big) < 2:
        return [region]
    K = len(big)
    dt = cv2.distanceTransform(np.pad(region, 1), cv2.DIST_L2, 3)[1:-1, 1:-1]
    cores = [(_fill((cl == i).astype(np.uint8) * 255) & region) > 0 for i in big]
    thick = np.array([2 * dt[c].max() for c in cores])
    bott = np.zeros((K, K))
    tmax = int(min(np.sort(thick)[-2] / 2, dt.max()))
    for t in range(1, tmax + 1):
        n, lab = cv2.connectedComponents((dt >= t).astype(np.uint8), connectivity=8)
        sets = [set(np.unique(lab[c & (dt >= t)]).tolist()) - {0} for c in cores]
        any_pair = False
        for i in range(K):
            for j in range(i + 1, K):
                if sets[i] & sets[j]:
                    bott[i, j] = bott[j, i] = 2 * t
                    any_pair = True
        if not any_pair:
            break
    parent = list(range(K))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for i in range(K):
        for j in range(i + 1, K):
            if bott[i, j] > 0 and bott[i, j] >= p["neck_ratio"] * np.sqrt(thick[i] * thick[j]):
                parent[find(i)] = find(j)
    roots = sorted({find(k) for k in range(K)})
    if len(roots) == 1:
        return [region]
    elev = (255 - np.clip(dt * (255.0 / max(dt.max(), 1)), 0, 255)).astype(np.uint8)
    markers = np.zeros(region.shape, np.int32)
    markers[region == 0] = len(roots) + 1
    for k in range(K):
        markers[cores[k]] = roots.index(find(k)) + 1
    cv2.watershed(cv2.cvtColor(elev, cv2.COLOR_GRAY2BGR), markers)
    ws = np.where((markers >= 1) & (markers <= len(roots)), markers, 0)
    ws_d = cv2.dilate(ws.astype(np.float32), np.ones((3, 3), np.uint8)).astype(np.int32)
    cell = np.where(region > 0, np.where(ws > 0, ws, ws_d), 0)
    parts = []
    for g in range(1, len(roots) + 1):
        m = (cell == g).astype(np.uint8) * 255
        tg = max(thick[k] for k in range(K) if roots.index(find(k)) + 1 == g)
        r = int(round(min(p["trim"] * tg, p["trim_max"] * np.sqrt(region.size))))
        if r >= 1:
            se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
            mo = cv2.morphologyEx(m, cv2.MORPH_OPEN, se)
            if cv2.countNonZero(mo) > 0.5 * cv2.countNonZero(m):
                m = mo
        parts.append(m)
    return parts


def _merge_contained(parts, frac):
    changed = True
    while changed and len(parts) > 1:
        changed = False
        areas = [cv2.countNonZero(m) for m in parts]
        order = np.argsort(areas)[::-1]
        for ia in order:
            cnts, _ = cv2.findContours(parts[ia], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            hull = cv2.convexHull(np.concatenate(cnts))
            hm = np.zeros_like(parts[ia])
            cv2.fillConvexPoly(hm, hull, 255)
            for ib in order:
                if ib == ia or areas[ib] > areas[ia]:
                    continue
                inside = cv2.countNonZero(parts[ib] & hm)
                if inside >= frac * areas[ib]:
                    parts[ia] = parts[ia] | parts[ib]
                    parts.pop(ib)
                    changed = True
                    break
            if changed:
                break
    return parts


def _make_det(part, x, y, Hinv, c, cand, D, paper_opp):
    h, w = part.shape
    cnts, _ = cv2.findContours(part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    pts = np.concatenate([cc.reshape(-1, 2) for cc in cnts]).astype(np.float32) + (x, y)
    img_pts = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), Hinv).reshape(-1, 2)
    x0, y0 = img_pts.min(0)
    x1, y1 = img_pts.max(0)
    mom = cv2.moments(part, binaryImage=True)
    cr = np.array([[[x + mom["m10"] / mom["m00"], y + mom["m01"] / mom["m00"]]]], np.float32)
    cx, cy = cv2.perspectiveTransform(cr, Hinv)[0, 0]
    sl = (slice(y, y + h), slice(x, x + w))
    evid = (part > 0) & (cand[sl] > 0)
    name = _name(c["Rb"][sl][evid], c["opp"][sl][evid], D[sl][evid], paper_opp)
    return {"cls": "object", "label": f"{name} object", "draw": DRAW.get(name, (200, 200, 200)),
            "img": (float(cx), float(cy)), "bbox": (float(x0), float(y0), float(x1 - x0), float(y1 - y0)),
            "contour": img_pts}


def _name(px_bgr, opp_px, D_px, paper_opp):
    if len(px_bgr) == 0:
        return "unknown"
    dc = opp_px - paper_opp
    colourful = np.sqrt((dc ** 2).sum(1)) > 0.18
    if colourful.mean() >= 0.20 and colourful.sum() >= 15:
        c = np.median(px_bgr[colourful], axis=0).astype(np.uint8).reshape(1, 1, 3)
        h, s, v = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)[0, 0]
        if h < 6 or h >= 170:
            return "red"
        if h < 16:
            return "brown" if v < 120 else "orange"
        if h < 36:
            return "yellow"
        if h < 85:
            return "green"
        if h < 130:
            return "blue"
        if h < 150:
            return "purple"
        return "pink"
    md = np.median(D_px)
    return "black" if md > 0.45 else "gray" if md > 0.2 else "white"


DRAW = {"red": (235, 80, 80), "green": (90, 220, 120), "blue": (90, 140, 255), "yellow": (240, 210, 70),
        "orange": (255, 160, 60), "brown": (170, 110, 60), "purple": (170, 100, 230), "pink": (255, 130, 200),
        "black": (60, 60, 60), "gray": (160, 160, 160), "white": (235, 235, 235), "unknown": (200, 200, 200)}


def _evidence(frame, work_quad, p, background=None):
    if work_quad is None:
        h, w = frame.shape[:2]
        work_quad = [(0, 0), (w - 1, 0), (w - 1, h - 1), (0, h - 1)]
    q, H, Wr, Hr, m, size = _geometry(work_quad, p)
    out_w, out_h = Wr + 2 * m, Hr + 2 * m
    R = cv2.warpPerspective(frame, H, (out_w, out_h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    c = _cues(R, p, size)
    D, C, sharp = c["D"], c["C"], c["sharp"]

    area = np.zeros((out_h, out_w), np.uint8)
    area[m:m + Hr, m:m + Wr] = 255
    ins = int(round(p["inset"] * size))
    inner = np.zeros_like(area)
    inner[m + ins:m + Hr - ins, m + ins:m + Wr - ins] = 255
    ink = _ink_mask(D, C, Wr, Hr, m, size, p)
    valid = inner & ~ink

    edge_any = ((sharp > p["sharp_cand"]).astype(np.uint8) * 255) | c["canny"]
    near_edge = cv2.dilate(edge_any, _rect_k(_odd(p["dark_edge_r"] * size))) > 0
    strong = (sharp > p["sharp_seed"]) | ((D > p["dark_seed"]) & near_edge) | (C > p["chroma_seed"])
    weak = (sharp > p["sharp_cand"]) | ((D > p["dark_cand"]) & near_edge) | (C > p["chroma_cand"])
    seed = strong.astype(np.uint8) * 255 & valid
    cand = (weak.astype(np.uint8) * 255 | c["canny"]) & valid
    weak_only = cand & ~cv2.dilate(seed, _rect_k(3))
    L = _odd(p["seam_len"] * size)
    wh = cv2.dilate(weak_only, cv2.getStructuringElement(cv2.MORPH_RECT, (1, 3)))
    wv = cv2.dilate(weak_only, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1)))
    seams = (cv2.morphologyEx(wh, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (L, 1))) |
             cv2.morphologyEx(wv, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (1, L)))) & weak_only
    cand &= ~cv2.dilate(seams, _rect_k(3))

    if background is not None and background.shape[:2] == frame.shape[:2]:
        Rb_ = cv2.warpPerspective(background, H, (out_w, out_h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        cb = _cues(Rb_, p, size)
        bg_ev = ((cb["sharp"] > p["sharp_cand"]) | (cb["D"] > p["dark_cand"]) | (cb["C"] > p["chroma_cand"]) |
                 (cb["canny"] > 0)).astype(np.uint8) * 255
        bg_ev = cv2.dilate(bg_ev, _rect_k(5)) > 0
        dD = cv2.blur(np.abs(D - cb["D"]), (5, 5))
        dC = cv2.blur(np.abs(C - cb["C"]), (5, 5))
        same = bg_ev & (dD < 0.06) & (dC < 0.05)
        seed[same] = 0
        cand[same] = 0
    return dict(p=p, q=q, H=H, Wr=Wr, Hr=Hr, m=m, size=size, R=R, c=c, area=area, inner=inner, ink=ink,
                valid=valid, seed=seed, cand=cand)


def _regions(E, seed, cand, debug=None):
    p, size, Wr, Hr, c = E["p"], E["size"], E["Wr"], E["Hr"], E["c"]
    valid, inner, D, C = E["valid"], E["inner"], c["D"], c["C"]
    cand_j = cv2.morphologyEx(cand, cv2.MORPH_CLOSE, _rect_k(_odd(p["join_k"] * size))) & valid
    a_area = Wr * Hr
    min_area, max_area = p["min_area"] * a_area, p["max_area"] * a_area
    min_seed = max(8, p["min_seed"] * a_area)
    seed_j = cv2.morphologyEx(seed, cv2.MORPH_CLOSE, _rect_k(_odd(p["core_k"] * size))) & cand_j
    n, lab, st, _ = cv2.connectedComponentsWithStats(cand_j, connectivity=8)
    n_seed = np.bincount(lab[seed > 0], minlength=n)
    n_cand = np.bincount(lab[cand > 0], minlength=n)
    ok = (n_seed >= min_seed) | (n_cand >= p["min_edge_px"] * a_area)
    ok[0] = False

    Hinv = np.linalg.inv(E["H"])
    paper_px = (D < 0.06) & (C < 0.06) & (inner > 0)
    paper_opp = np.median(c["opp"][paper_px], axis=0) if paper_px.sum() > 50 else np.zeros(2, np.float32)
    parts = []
    for gi in np.nonzero(ok)[0]:
        x, y, w, h, _ = st[gi]
        sub = (lab[y:y + h, x:x + w] == gi).astype(np.uint8) * 255
        filled = _fill(sub)
        if cv2.countNonZero(filled) < min_area:
            continue
        for part in _split(filled, seed_j[y:y + h, x:x + w], seed[y:y + h, x:x + w], min_seed, p):
            part = _fill(part)
            if cv2.countNonZero(part) < 0.3 * min_area:
                continue
            full = np.zeros_like(cand_j)
            full[y:y + h, x:x + w] = part
            parts.append(full)
    parts = _merge_contained(parts, p["contain_frac"])
    fg = np.zeros_like(cand_j)
    found = []
    for full in parts:
        x, y, w, h = cv2.boundingRect(full)
        part = full[y:y + h, x:x + w]
        a = cv2.countNonZero(part)
        if a < min_area or a > max_area:
            continue
        ps = cv2.countNonZero(part & seed[y:y + h, x:x + w])
        pc = cv2.countNonZero(part & cand[y:y + h, x:x + w])
        if ps < min_seed and pc < p["min_edge_px"] * a_area:
            continue
        dt = cv2.distanceTransform(cv2.copyMakeBorder(part, 1, 1, 1, 1, cv2.BORDER_CONSTANT), cv2.DIST_L2, 3)
        if dt.max() < p["min_thick"] * size:
            continue
        fg |= full
        found.append(_make_det(part, x, y, Hinv, c, cand, D, paper_opp))
    found.sort(key=lambda d: (d["img"][1], d["img"][0]))
    if debug is not None:
        debug.update(R=E["R"], cues=c, ink=E["ink"], valid=valid, seed=seed, cand=cand, cand_j=cand_j, fg=fg,
                     H=E["H"], geom=(Wr, Hr, E["m"], size))
    return found


def detect_objects(frame, work_quad, background=None, params=None, debug=None):
    p = dict(P, **(params or {}))
    E = _evidence(frame, work_quad, p, background)
    return _regions(E, E["seed"], E["cand"], debug)


class ObjectDetector:

    def __init__(self, alpha=0.25, keep=0.25, params=None):
        self.alpha, self.keep, self.params = alpha, keep, params
        self.reset()

    def reset(self):
        self._key, self._seed, self._cand = None, None, None

    def __call__(self, frame, work_quad, background=None, debug=None):
        p = dict(P, **(self.params or {}))
        E = _evidence(frame, work_quad, p, background)
        key = (frame.shape[:2], tuple(np.round(E["q"].ravel() / 3.0).astype(int)))
        s = E["seed"].astype(np.float32) / 255.0
        cnd = E["cand"].astype(np.float32) / 255.0
        if key != self._key or self._seed is None or self._seed.shape != s.shape:
            self._key, self._seed, self._cand = key, s, cnd
        else:
            a = self.alpha
            self._seed = a * s + (1 - a) * self._seed
            self._cand = a * cnd + (1 - a) * self._cand
        seed = (self._seed > self.keep).astype(np.uint8) * 255 & E["valid"]
        cand = ((self._cand > self.keep).astype(np.uint8) * 255 | seed) & E["valid"]
        return _regions(E, seed, cand, debug)
