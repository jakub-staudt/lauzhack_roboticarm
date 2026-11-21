"""Find the left rectangle of the table in an overhead camera frame, from AprilTags and the thin black lines.

The paper has two horizontal lines, three vertical lines (left, middle, right) and AprilTags (family 36h11) in the
corners. The left rectangle is the 20 x 20 cm square between the left and middle vertical lines and the two horizontal
ones. The robot arm often hides part of it, so the detector does not need every line:

* tags tell which side of the paper is which, and which horizontal lines belong to the rectangle (the ones between the
  top and bottom tags; the paper edges outside them are ignored);
* thin dark lines are found with a black-hat filter and a Hough transform; thick dark things (cables, motors) are removed;
* the left edge is the detected left line if it is visible, otherwise it is *estimated* from the other two vertical lines
  (the left and right squares have the same size) and the result says so.

`detect(bgr)` returns a plain dict (JSON-friendly) with every candidate it saw and the rectangle it chose, so the web page
can draw what the detector believes and the user can judge it.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from lauzkach_eye_robot_control.grid import RIGHT_SQUARE_CM, SIDE_CM as GRID_SIDE_CM, WORLD_CORNERS, fit_grid

TAG_DICT = cv2.aruco.DICT_APRILTAG_36h11
SIDE_CM = 20.0

# line filter
BLACKHAT_KERNEL = 9          # px: dark structures narrower than this are kept (lines), wider ones are cables / motors
BLACKHAT_THRESHOLD = 18      # low on purpose: the paper lines are 1 px wide and JPEG makes them faint and dotted
MAX_GRID_RMS_MM = 6.0         # a right-angled grid fits to about 1 mm; above this the lines are not a grid
MAX_LINE_THICKNESS = 2.6     # px (half width as a distance transform value); thicker dark things are not table lines
HOUGH_MIN_LENGTH = 40
HOUGH_MAX_GAP = 30
HOUGH_VOTES = 20
BRIDGE = 9                   # px: gaps this short along the picture's axes are filled, so dotted lines become continuous
ANGLE_TOLERANCE = math.radians(28)   # a "horizontal" line may be this far from horizontal (perspective, lens)
MERGE_ANGLE = math.radians(5)
MERGE_DISTANCE = 8.0         # px
MIN_SUPPORT = 70.0           # px of a line that is really covered by dark pixels
TAG_MARGIN = 12              # px around a tag that is ignored by the line finder
LEFT_AGREEMENT_PX = 20       # a detected left line must be within this of the grid's prediction at both ends (the right square's width is only known to +-1 cm)...
MIN_PAPER_LINE_BRIGHTNESS = 25   # a printed line is dark grey (about 35-110 in the picture); a cable or motor is nearly black
LEFT_ANGLE_TOLERANCE = math.radians(3.0)   # ...and lean the same way (the three vertical lines converge: an arm edge does not)
# Where the vertical lines sit between the left and right tag columns on the RAMION table (fraction of the distance
# from the left tags' x to the right tags' x, measured on the real table). Used to name the lines when both tag columns show.
LINE_POSITION = {"left": 0.03, "mid": 0.47, "right": 0.94}
LINE_POSITION_TOLERANCE = 0.13


# ---------------------------------------------------------------------------------------------- geometry helpers
def _h(p) -> np.ndarray:
    return np.array([p[0], p[1], 1.0])


def _line_through(p, q) -> np.ndarray:
    return np.cross(_h(p), _h(q))


def _intersect(l1: np.ndarray, l2: np.ndarray):
    p = np.cross(l1, l2)
    if abs(p[2]) < 1e-9:
        return None
    return float(p[0] / p[2]), float(p[1] / p[2])


class _Line:
    """A fitted straight line with the extent over which dark pixels were found."""

    def __init__(self, kind: str, centre: np.ndarray, direction: np.ndarray, t0: float, t1: float, support: float, coverage: float):
        self.kind, self.c, self.d = kind, centre, direction / np.linalg.norm(direction)
        self.t0, self.t1, self.support, self.coverage = t0, t1, support, coverage
        self.p0 = self.c + self.d * t0
        self.p1 = self.c + self.d * t1
        self.hom = _line_through(self.c, self.c + self.d)

    def t_of(self, p) -> float:
        return float(np.dot(np.asarray(p, float) - self.c, self.d))

    def y_at_x(self, x: float) -> float:
        return float(self.c[1] + (x - self.c[0]) * self.d[1] / self.d[0])

    def x_at_y(self, y: float) -> float:
        return float(self.c[0] + (y - self.c[1]) * self.d[0] / self.d[1])

    def public(self, name: str = "") -> dict:
        return {"kind": self.kind, "name": name, "p0": [round(float(v), 1) for v in self.p0],
                "p1": [round(float(v), 1) for v in self.p1], "support": round(self.support), "coverage": round(self.coverage, 2)}


# ---------------------------------------------------------------------------------------------- tags
def detect_tags(gray: np.ndarray) -> list[dict]:
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(TAG_DICT), params)
    corners, ids, _rejected = detector.detectMarkers(gray)
    tags = []
    for quad, tag_id in zip(corners, [] if ids is None else ids.ravel(), strict=False):
        quad = quad.reshape(4, 2)
        tags.append({"id": int(tag_id), "corners": [[round(float(x), 1), round(float(y), 1)] for x, y in quad],
                     "center": [round(float(quad[:, 0].mean()), 1), round(float(quad[:, 1].mean()), 1)]})
    return tags


# ---------------------------------------------------------------------------------------------- lines
def _thin_line_mask(gray: np.ndarray, tags: list[dict]) -> np.ndarray:
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (BLACKHAT_KERNEL, BLACKHAT_KERNEL))
    blackhat = cv2.morphologyEx(cv2.GaussianBlur(gray, (3, 3), 0), cv2.MORPH_BLACKHAT, kernel)
    mask = (blackhat > BLACKHAT_THRESHOLD).astype(np.uint8)
    # thick dark things (cables, motors, the logo) are not table lines: remove them together with a margin around them
    distance = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
    thick = cv2.dilate((distance > MAX_LINE_THICKNESS).astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13)))
    mask[thick > 0] = 0
    for tag in tags:  # the tag's own black and white pattern is full of short lines
        cv2.fillConvexPoly(mask, cv2.convexHull(np.array(tag["corners"], np.float32)).astype(np.int32), 0)
        box = cv2.boundingRect(np.array(tag["corners"], np.float32))
        x, y, w, h = box
        mask[max(0, y - TAG_MARGIN):y + h + TAG_MARGIN, max(0, x - TAG_MARGIN):x + w + TAG_MARGIN] = 0
    bridged = cv2.max(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((BRIDGE, 1), np.uint8)),
                      cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((1, BRIDGE), np.uint8)))
    return cv2.max(mask, bridged)


def _fit(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    vx, vy, x0, y0 = cv2.fitLine(points.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    return np.array([x0, y0], float), np.array([vx, vy], float)


def find_lines(gray: np.ndarray, tags: list[dict]) -> tuple[list[_Line], np.ndarray]:
    mask = _thin_line_mask(gray, tags)
    segments = cv2.HoughLinesP(mask * 255, 1, np.pi / 360, HOUGH_VOTES, minLineLength=HOUGH_MIN_LENGTH, maxLineGap=HOUGH_MAX_GAP)
    clusters: list[dict] = []
    for x1, y1, x2, y2 in ([] if segments is None else np.asarray(segments).reshape(-1, 4)):  # OpenCV 4 gives (N,1,4), 5 gives (N,4)
        angle = math.atan2(y2 - y1, x2 - x1) % math.pi
        horizontal = angle < ANGLE_TOLERANCE or angle > math.pi - ANGLE_TOLERANCE
        vertical = abs(angle - math.pi / 2) < ANGLE_TOLERANCE
        if not (horizontal or vertical):
            continue
        kind = "H" if horizontal else "V"
        ends = np.array([[x1, y1], [x2, y2]], float)
        for cluster in clusters:
            if cluster["kind"] != kind:
                continue
            c, d = cluster["fit"]
            da = abs(((angle - cluster["angle"] + math.pi / 2) % math.pi) - math.pi / 2)
            dist = np.abs((ends - c) @ np.array([-d[1], d[0]]))
            if da < MERGE_ANGLE and dist.max() < MERGE_DISTANCE:
                cluster["points"] = np.vstack([cluster["points"], ends])
                cluster["fit"] = _fit(cluster["points"])
                cluster["angle"] = math.atan2(cluster["fit"][1][1], cluster["fit"][1][0]) % math.pi
                break
        else:
            clusters.append({"kind": kind, "points": ends, "fit": _fit(ends), "angle": angle})

    near = cv2.dilate(mask, np.ones((3, 3), np.uint8))
    lines = []
    for cluster in clusters:
        c, d = cluster["fit"]
        if d[0] < 0 or (d[0] == 0 and d[1] < 0):
            d = -d
        t = (cluster["points"] - c) @ d
        t0, t1 = float(t.min()), float(t.max())
        samples = c[None, :] + d[None, :] * np.arange(t0, t1, 1.0)[:, None]
        xs, ys = np.round(samples[:, 0]).astype(int), np.round(samples[:, 1]).astype(int)
        inside = (xs >= 0) & (xs < gray.shape[1]) & (ys >= 0) & (ys < gray.shape[0])
        hit = np.zeros(len(samples), bool)
        hit[inside] = near[ys[inside], xs[inside]] > 0
        support = float(hit.sum())
        if support >= MIN_SUPPORT:
            lines.append(_Line(cluster["kind"], c, d, t0, t1, support, support / max(1.0, t1 - t0)))
    return lines, mask


# ---------------------------------------------------------------------------------------------- the rectangle
def _darkest_across(gray: np.ndarray, line: _Line) -> float:
    """Median over the line's length of the darkest pixel within 2 px of it: printed lines are mid-grey, cables are black."""
    n = int(line.t1 - line.t0)
    if n < 6:
        return 255.0
    normal = np.array([-line.d[1], line.d[0]])
    values = []
    for t in np.arange(line.t0, line.t1, 3.0):
        centre = line.c + line.d * t
        best = 255
        for offset in (-2, -1, 0, 1, 2):
            x, y = np.round(centre + normal * offset).astype(int)
            if 0 <= x < gray.shape[1] and 0 <= y < gray.shape[0]:
                best = min(best, int(gray[y, x]))
        values.append(best)
    return float(np.median(values))


def _split_tags(tags: list[dict], width: int) -> tuple[list[dict], list[dict]]:
    """(left tags, right tags): tags on either side of the middle of the tag span, or of the image when only one side shows."""
    if not tags:
        return [], []
    xs = [t["center"][0] for t in tags]
    split = (min(xs) + max(xs)) / 2 if max(xs) - min(xs) > 0.25 * width else width / 2
    return [t for t in tags if t["center"][0] < split], [t for t in tags if t["center"][0] >= split]


def _harmonic_left(x_mid: float, x_right: float, vanishing: float | None) -> float:
    """Position of the left line along a horizontal line, given the middle and right lines at equal world spacing."""
    if vanishing is None or abs(vanishing - x_mid) > 1e6 or abs(vanishing - x_right) > 1e6:
        return 2 * x_mid - x_right
    a, b = 1.0 / (vanishing - x_mid), 1.0 / (vanishing - x_right)
    inverse = 2 * a - b
    return 2 * x_mid - x_right if abs(inverse) < 1e-9 else vanishing - 1.0 / inverse


def choose_rectangle(tags: list[dict], lines: list[_Line], shape: tuple[int, int], gray: np.ndarray | None = None,
                     mask: np.ndarray | None = None, right_cm: float = RIGHT_SQUARE_CM) -> dict:
    height, width = shape[:2]
    issues: list[str] = []
    result: dict = {"polygon": None, "method": None, "issues": issues, "chosen": {}}
    left_tags, right_tags = _split_tags(tags, width)
    horizontal = [l for l in lines if l.kind == "H"]
    vertical = [l for l in lines if l.kind == "V"]

    # 1. the two horizontal lines: the strongest ones between the top and bottom tags
    if len(tags) < 2:
        issues.append(f"only {len(tags)} tag(s) visible; need two tags in one column to tell which lines belong to the rectangle")
        return result
    column = max((left_tags, right_tags), key=lambda c: (len(c), 0))
    ys = sorted(t["center"][1] for t in column)
    if len(column) < 2:
        issues.append("no column of two tags is visible")
        return result
    band_top, band_bottom = ys[0], ys[-1]
    ref_x = float(np.mean([t["center"][0] for t in column]))
    inside = [l for l in horizontal if band_top + 8 < l.y_at_x(ref_x) < band_bottom - 8]
    inside.sort(key=lambda l: -l.support)
    if len(inside) < 2:
        issues.append(f"found {len(inside)} horizontal line(s) between the tags, need 2")
        return result
    pair = sorted(inside[:2], key=lambda l: l.y_at_x(ref_x))
    top, bottom = pair
    result["chosen"]["top"], result["chosen"]["bottom"] = top, bottom

    # 2. vertical lines that cross both horizontal ones inside their detected extent
    def crossing(line: _Line, h: _Line):
        p = _intersect(line.hom, h.hom)
        if p is None:
            return None
        slack = 150  # the lens bends long lines a little, so a line's detected pieces can stop short of the crossing
        return p if h.t0 - slack <= h.t_of(p) <= h.t1 + slack else None

    crossers = []
    for v in vertical:
        pt, pb = crossing(v, top), crossing(v, bottom)
        if pt and pb and 0.5 * (bottom.y_at_x(ref_x) - top.y_at_x(ref_x)) < math.dist(pt, pb) < 2.5 * (bottom.y_at_x(ref_x) - top.y_at_x(ref_x)):
            crossers.append((0.5 * (pt[0] + pb[0]), v, pt, pb))
    crossers.sort(key=lambda c: c[0])
    result["crossers"] = [c[1] for c in crossers]
    if len(crossers) < 2:
        issues.append(f"found {len(crossers)} vertical line(s) crossing the rectangle, need at least 2")
        return result

    # 3. name the vertical lines. With both tag columns in view, by where each line sits between them; with one column,
    #    the line nearest that column is the outermost one.
    names: dict[str, tuple] = {}
    if left_tags and right_tags and (xr := np.mean([t["center"][0] for t in right_tags])) - (xl := np.mean([t["center"][0] for t in left_tags])) > 0.3 * width:
        taken: set[int] = set()
        for role in ("mid", "right", "left"):
            options = sorted(((abs((c[0] - xl) / (xr - xl) - LINE_POSITION[role]), i) for i, c in enumerate(crossers)
                              if i not in taken), key=lambda o: o[0])
            if options and options[0][0] <= LINE_POSITION_TOLERANCE:
                names[role] = crossers[options[0][1]]
                taken.add(options[0][1])
    elif right_tags and not left_tags:
        ordered = crossers[-3:]
        names = dict(zip(("right", "mid", "left")[:len(ordered)], reversed(ordered), strict=False))
    elif left_tags and not right_tags:
        ordered = crossers[:3]
        names = dict(zip(("left", "mid", "right")[:len(ordered)], ordered, strict=False))
    if "mid" not in names:
        issues.append("could not identify the middle vertical line")
        return result
    mid = names["mid"]
    result["chosen"]["mid"] = mid[1]
    if "right" in names:
        result["chosen"]["right"] = names["right"][1]

    # 4. the left edge. Estimated from the middle and right lines (equal square size) when those are visible; a detected
    #    left line is trusted over the estimate when it is plausible (right place, leans like the other vertical lines,
    #    not pitch black); the estimate is only a rough fallback, measured 1-2 cm off on the real table.
    estimate = None
    if "right" in names:
        vanish = _intersect(top.hom, bottom.hom)
        points = {}
        for label, hline, k in (("top", top, 2), ("bottom", bottom, 3)):
            tv = hline.t_of(vanish) if vanish is not None else None
            t_left = _harmonic_left(hline.t_of(names["mid"][k]), hline.t_of(names["right"][k]), tv)
            points[label] = tuple(float(v) for v in hline.c + hline.d * t_left)
        estimate = (points["top"], points["bottom"])
        if mask is not None:   # better: the right-angle grid fit with the known (measured) width of the right square
            rough = np.array([points["top"], (names["mid"][2]), (names["mid"][3]), points["bottom"]], float)
            pixels = {n: _line_pixels(mask, l) for n, l in (("top", top), ("bottom", bottom), ("mid", names["mid"][1]), ("right", names["right"][1]))}
            fit = fit_grid(pixels, rough, shape, right_cm)
            if fit is not None and fit.rms_mm <= MAX_GRID_RMS_MM:
                c = fit.corners_px()
                estimate = (tuple(float(v) for v in c[0]), tuple(float(v) for v in c[3]))
    detected = (names["left"][2], names["left"][3]) if "left" in names else None
    if detected and gray is not None and _darkest_across(gray, names["left"][1]) < MIN_PAPER_LINE_BRIGHTNESS:
        issues.append("a nearly black line at the left edge was ignored (a cable or a motor, not the printed line)")
        names.pop("left")
        detected = None
    result["left_estimate"] = None if estimate is None else [list(map(lambda v: round(v, 1), q)) for q in estimate]
    result["left_detected"] = None if detected is None else [list(map(lambda v: round(v, 1), q)) for q in detected]
    if detected and estimate:
        gap = max(math.dist(detected[0], estimate[0]), math.dist(detected[1], estimate[1]))
        lean = abs(math.atan2(detected[1][0] - detected[0][0], detected[1][1] - detected[0][1])
                   - math.atan2(estimate[1][0] - estimate[0][0], estimate[1][1] - estimate[0][1]))
        if gap <= LEFT_AGREEMENT_PX and lean <= LEFT_ANGLE_TOLERANCE:
            (tl, bl), method = detected, "left line detected (printed line seen)"
            result["chosen"]["left"] = names["left"][1]
        else:
            (tl, bl), method = estimate, "left edge estimated from the middle and right lines"
            issues.append(f"a line at the left edge was ignored: {gap:.0f} px and {math.degrees(lean):.0f} degrees from the estimate (probably the arm)")
    elif estimate:
        (tl, bl), method = estimate, "left edge estimated from the middle and right lines"
        issues.append("left edge is estimated (the left line is not visible); the estimate can be 1-2 cm off, check it against the paper")
    elif detected:
        issues.append("a line was seen at the left but there is no right line to check it against, so it is not trusted (it may be the arm)")
        return result
    else:
        issues.append("neither the left nor the right vertical line is visible: cannot place the left edge")
        return result

    tr, br = mid[2], mid[3]
    polygon = [tl, tr, br, bl]
    result.update(polygon=[[round(float(x), 1), round(float(y), 1)] for x, y in polygon], method=method)
    # sanity checks: convex, inside the picture (a little outside is fine), roughly square
    quad = np.array(polygon, np.float32)
    if not cv2.isContourConvex(quad.reshape(-1, 1, 2)):
        issues.append("the rectangle is not convex")
    margin = 40
    if (quad[:, 0] < -margin).any() or (quad[:, 0] > width + margin).any() or (quad[:, 1] < -margin).any() or (quad[:, 1] > height + margin).any():
        issues.append("part of the rectangle is outside the picture")
    sides = [math.dist(polygon[i], polygon[(i + 1) % 4]) for i in range(4)]
    result["side_px"] = [round(s, 1) for s in sides]
    ratio = (sides[0] + sides[2]) / (sides[1] + sides[3])
    result["aspect"] = round(ratio, 2)
    if not 0.7 < ratio < 1.4:
        issues.append(f"the rectangle is not square (width/height {ratio:.2f}); it should be about 1")
    return result


def rectify(bgr: np.ndarray, polygon: list[list[float]], size: int = 360) -> np.ndarray:
    """The rectangle seen from straight above, `size` x `size` px (TL, TR, BR, BL order)."""
    target = np.array([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]], np.float32)
    matrix = cv2.getPerspectiveTransform(np.array(polygon, np.float32), target)
    return cv2.warpPerspective(bgr, matrix, (size, size), flags=cv2.INTER_LINEAR)


def _line_pixels(mask: np.ndarray, line: _Line) -> np.ndarray:
    """The thin-line pixels that lie on `line` (within 2 px, inside its detected extent), as (N, 2) x, y."""
    ys, xs = np.nonzero(mask)
    pixels = np.stack([xs, ys], 1).astype(float)
    rel = pixels - line.c
    keep = (np.abs(rel @ np.array([-line.d[1], line.d[0]])) < 2.0) & (rel @ line.d > line.t0 - 2) & (rel @ line.d < line.t1 + 2)
    return pixels[keep]


def _apply_grid(chosen: dict, mask: np.ndarray, shape: tuple[int, int], right_cm: float) -> dict | None:
    """Refine the rectangle with the right-angle model. Returns grid info (and updates `chosen`), or None if no fit was possible."""
    lines = chosen["chosen"]
    points = {name: _line_pixels(mask, line) for name, line in lines.items()}
    init = np.array(chosen["polygon"], float)
    left_seen = "left" in lines
    own = ("top", "bottom", "left", "mid") if left_seen else ("top", "bottom", "mid", "right")   # the lines of the square itself
    fit = fit_grid({n: points[n] for n in own if n in points}, init, shape, right_cm)
    info: dict = {"right_cm_assumed": right_cm, "lines": [n for n in own if n in points]}
    if fit is None or fit.rms_mm > MAX_GRID_RMS_MM:
        info["rms_mm"] = None if fit is None else round(fit.rms_mm, 2)
        chosen["issues"].append("the detected lines do not fit one right-angled grid"
                                + ("" if fit is None else f" (worst {max(fit.per_line_mm, key=fit.per_line_mm.get)}: "
                                                            f"{max(fit.per_line_mm.values()):.1f} mm)") + "; using the plain line intersections")
        return info
    corners = fit.corners_px()
    chosen["polygon"] = [[round(float(x), 1), round(float(y), 1)] for x, y in corners]
    info.update(rms_mm=round(fit.rms_mm, 2), per_line_mm={k: round(v, 2) for k, v in fit.per_line_mm.items()})
    n = np.array([[1 / fit.scale, 0, -fit.centre[0] / fit.scale], [0, 1 / fit.scale, -fit.centre[1] / fit.scale], [0, 0, 1]])
    info["px_to_cm"] = (fit.matrix @ n).tolist()
    if left_seen and "right" in points and len(points["right"]) >= 8:   # what the grid says about the right square
        info["right_width_cm"] = round(float(np.mean(fit.to_cm(points["right"])[:, 0])) - GRID_SIDE_CM, 2)
    # the left edge predicted from the other four lines, for the page to compare with the line that was seen
    if "right" in points:
        pred = fit_grid({n: points[n] for n in ("top", "bottom", "mid", "right") if n in points}, init, shape, right_cm)
        if pred is not None and pred.rms_mm <= MAX_GRID_RMS_MM:
            c = pred.corners_px()
            chosen["left_estimate"] = [[round(float(c[0][0]), 1), round(float(c[0][1]), 1)], [round(float(c[3][0]), 1), round(float(c[3][1]), 1)]]
    return info


def detect(bgr: np.ndarray, right_cm: float = RIGHT_SQUARE_CM) -> dict:
    """Everything the detector sees in one frame, as plain Python data. `right_cm`: width of the right square (it is not 20)."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    tags = detect_tags(gray)
    lines, mask = find_lines(gray, tags)
    chosen = choose_rectangle(tags, lines, gray.shape, gray, mask, right_cm)
    grid = _apply_grid(chosen, mask, gray.shape, right_cm) if chosen["polygon"] else None
    if grid and grid.get("rms_mm") is not None and "left" not in chosen["chosen"]:
        chosen["method"] = f"left edge estimated from the grid (right square assumed {right_cm:g} cm wide)"
    names = {id(l): n for n, l in chosen["chosen"].items()}
    out = {
        "width": int(gray.shape[1]), "height": int(gray.shape[0]),
        "tags": tags,
        "lines": [l.public(names.get(id(l), "")) for l in sorted(lines, key=lambda l: -l.support)],
        "chosen": {name: l.public(name) for name, l in chosen["chosen"].items()},
        "polygon": chosen["polygon"], "method": chosen["method"], "issues": chosen["issues"],
        "aspect": chosen.get("aspect"), "side_px": chosen.get("side_px"), "grid": grid,
        "left_estimate": chosen.get("left_estimate"), "left_detected": chosen.get("left_detected"),
        "ok": chosen["polygon"] is not None and not any(("not convex" in i or "not square" in i or "outside" in i) for i in chosen["issues"]),
    }
    return out
