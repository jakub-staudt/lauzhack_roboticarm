"""Automatic, read-only cached-camera collection and tentative planar registration."""
import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path
import signal
import threading
import time
import urllib.request

import cv2
import numpy as np


def atomic_json(path, data):
    stage = path.with_suffix('.next')
    stage.write_text(json.dumps(data, indent=2) + '\n')
    stage.replace(path)


def atomic_image(path, image):
    ok, encoded = cv2.imencode(path.suffix, image)
    if not ok:
        raise ValueError('Image encoding failed')
    stage = path.with_suffix('.next')
    stage.write_bytes(encoded.tobytes())
    stage.replace(path)


def register_plane(reference, candidate):
    """Candidate-to-anchor pixels; reject weak matches rather than fabricate coverage."""
    sift = cv2.SIFT_create(nfeatures=4000)
    a, da = sift.detectAndCompute(cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY), None)
    b, db = sift.detectAndCompute(cv2.cvtColor(candidate, cv2.COLOR_BGR2GRAY), None)
    result = {'accepted': False, 'reference_features': len(a), 'candidate_features': len(b)}
    if da is None or db is None or len(a) < 12 or len(b) < 12:
        return None, {**result, 'reason': 'too_few_features'}
    pairs = cv2.BFMatcher().knnMatch(db, da, k=2)
    matches = [pair[0] for pair in pairs if len(pair) == 2 and pair[0].distance < .7 * pair[1].distance]
    result['ratio_matches'] = len(matches)
    if len(matches) < 12:
        return None, {**result, 'reason': 'too_few_matches'}
    src = np.float32([b[m.queryIdx].pt for m in matches])
    dst = np.float32([a[m.trainIdx].pt for m in matches])
    h, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    if h is None or mask is None:
        return None, {**result, 'reason': 'no_homography'}
    keep = mask.ravel().astype(bool)
    result.update(inliers=int(keep.sum()), inlier_fraction=float(keep.mean()))
    if keep.sum() < 12 or keep.mean() < .5:
        return None, {**result, 'reason': 'weak_consensus'}
    coverage = []
    for points, image in [(src[keep], candidate), (dst[keep], reference)]:
        coverage.append(float(cv2.contourArea(cv2.convexHull(points))) / (image.shape[0] * image.shape[1]))
    result['feature_hull_fraction'] = coverage
    if min(coverage) < .03:
        return None, {**result, 'reason': 'matches_too_localized'}
    corners = np.float32([[0, 0], [candidate.shape[1]-1, 0], [candidate.shape[1]-1, candidate.shape[0]-1], [0, candidate.shape[0]-1]])
    denominators = corners @ h[2, :2] + h[2, 2]
    if not np.isfinite(h).all() or np.any(abs(denominators) < 1e-8) or not np.all(np.sign(denominators) == np.sign(denominators[0])):
        return None, {**result, 'reason': 'invalid_projection'}
    warped = cv2.perspectiveTransform(corners[:, None], h)[:, 0]
    area = abs(cv2.contourArea(warped.astype(np.float32))) / (reference.shape[0] * reference.shape[1])
    if not cv2.isContourConvex(warped.astype(np.float32)) or not .01 < area < 16 or np.max(abs(warped)) > 4000:
        return None, {**result, 'reason': 'implausible_footprint'}
    projected = cv2.perspectiveTransform(src[:, None], h)[:, 0]
    error = np.linalg.norm(projected[keep] - dst[keep], axis=1)
    result.update(accepted=True, reason='candidate_planar_alignment', reprojection_p95_px=float(np.percentile(error, 95)), candidate_to_anchor=h.tolist())
    return h, result


def render_mosaic(anchor, layers):
    corners = [np.float32([[0, 0], [anchor.shape[1], 0], [anchor.shape[1], anchor.shape[0]], [0, anchor.shape[0]]])]
    for image, h in layers:
        p = np.float32([[0, 0], [image.shape[1], 0], [image.shape[1], image.shape[0]], [0, image.shape[0]]])
        corners.append(cv2.perspectiveTransform(p[:, None], h)[:, 0])
    points = np.concatenate(corners)
    low = np.floor(points.min(axis=0)).astype(int)
    high = np.ceil(points.max(axis=0)).astype(int)
    width, height = high-low
    if width > 4096 or height > 4096 or width * height > 8_000_000:
        raise ValueError('Mosaic bounds exceed collection budget')
    translate = np.array([[1., 0., -low[0]], [0., 1., -low[1]], [0., 0., 1.]])
    mosaic = np.zeros((height, width, 3), np.uint8)
    covered = np.zeros((height, width), np.uint8)
    # Lay registered details down first, then anchor; overlapping pixels are not new coverage.
    for image, h in layers + [(anchor, np.eye(3))]:
        transform = translate @ h
        warped = cv2.warpPerspective(image, transform, (width, height))
        mask = cv2.warpPerspective(np.full(image.shape[:2], 255, np.uint8), transform, (width, height), flags=cv2.INTER_NEAREST)
        mosaic[mask > 0] = warped[mask > 0]
        covered = np.maximum(covered, mask)
    return mosaic, {'anchor_pixel_bounds': [low.tolist(), high.tolist()], 'observed_output_pixels': int(np.count_nonzero(covered)), 'unobserved_output_pixels': int(covered.size-np.count_nonzero(covered)), 'coverage_units': 'anchor_camera_pixels_not_mm'}


def fetch(base, route):
    start = time.monotonic()
    with urllib.request.urlopen(base + route, timeout=3) as response:
        data = response.read(2_000_001)
        if len(data) > 2_000_000:
            raise ValueError('Capture too large')
    return data, round((time.monotonic()-start)*1000, 3)


def collect(base, output, seconds=60, interval=1., stop=None):
    cv2.setNumThreads(1)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    stop = stop or threading.Event()
    start = time.monotonic()
    status = {'phase': 'collecting', 'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), 'elapsed_s': 0, 'duration_s': seconds, 'captured_frames': 0, 'registered_frames': 0, 'rejected_frames': 0, 'duplicate_frames': 0, 'errors': 0, 'units': 'pixels', 'metric_calibrated': False, 'robot_motion_commands': 0, 'registration_validated': False, 'note': 'Automatic camera collection; unknown areas and scale are not inferred.'}
    atomic_json(output/'status.json', status)
    anchor = None
    layers = []
    seen = set()
    # Raw frames stay on the server; no keys, ownership tokens or images go to GitHub.
    with (output/'frames.jsonl').open('w') as log, concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        frame_id = 0
        while time.monotonic()-start < seconds and not stop.is_set():
            tick = time.monotonic()
            state_job = pool.submit(fetch, base, '/api/state')
            jobs = {name: pool.submit(fetch, base, '/cameras/'+name+'.jpg') for name in ['overhead', 'wrist']}
            try:
                data, state_ms = state_job.result()
                raw = json.loads(data)
                state = {k: raw.get(k) for k in ['connected', 'loop_ok', 'age_s', 'sequence', 'measured', 'faults']}
                state['request_ms'] = state_ms
            except Exception as exc:
                state = {'error': str(exc)}
            for name, job in jobs.items():
                row = {'index': frame_id, 'camera': name, 'elapsed_s': round(time.monotonic()-start, 3), 'controller': state, 'synchronized_exposure': False}
                try:
                    data, elapsed = job.result()
                    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                    if image is None:
                        raise ValueError('Invalid JPEG')
                    filename = f'{frame_id:05d}-{name}.jpg'
                    (output/filename).write_bytes(data)
                    atomic_image(output/(name+'.jpg'), image)
                    digest = hashlib.sha256(data).hexdigest()
                    row.update(file=filename, sha256=digest, request_ms=elapsed, width=image.shape[1], height=image.shape[0], sharpness=float(cv2.Laplacian(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()))
                    status['captured_frames'] += 1
                    if (name, digest) in seen:
                        status['duplicate_frames'] += 1
                        row['registration'] = {'accepted': False, 'reason': 'identical_cached_frame'}
                    elif anchor is None and name == 'overhead':
                        anchor = image.copy()
                        row['registration'] = {'accepted': True, 'reason': 'anchor_view'}
                        status['registered_frames'] += 1
                    elif anchor is not None:
                        h, result = register_plane(anchor, image)
                        row['registration'] = result
                        if h is not None:
                            status['registered_frames'] += 1
                            # Bound memory and preserve only a limited set of mosaic candidates.
                            if len(layers) < 16:
                                footprint = cv2.perspectiveTransform(np.float32([[[0, 0]], [[image.shape[1], 0]], [[image.shape[1], image.shape[0]]], [[0, image.shape[0]]]]), h)[:, 0]
                                anchor_outline = np.float32([[0, 0], [anchor.shape[1], 0], [anchor.shape[1], anchor.shape[0]], [0, anchor.shape[0]]])
                                overlap, _ = cv2.intersectConvexConvex(footprint.astype(np.float32), anchor_outline)
                                new_area = abs(cv2.contourArea(footprint.astype(np.float32))) - overlap
                                if new_area > .03 * anchor.shape[0] * anchor.shape[1]:
                                    layers.append((image.copy(), h))
                        else:
                            status['rejected_frames'] += 1
                    seen.add((name, digest))
                except Exception as exc:
                    row['error'] = str(exc)
                    status['errors'] += 1
                log.write(json.dumps(row)+'\n')
                log.flush()
            if anchor is not None:
                try:
                    mosaic, coverage = render_mosaic(anchor, layers)
                    atomic_image(output/'mosaic.jpg', mosaic)
                    status['coverage'] = coverage
                    status['candidate_mosaic_layers'] = len(layers) + 1
                except ValueError as exc:
                    status['mosaic_error'] = str(exc)
            status.update(elapsed_s=round(time.monotonic()-start, 2), capture_cycles=frame_id+1)
            atomic_json(output/'status.json', status)
            frame_id += 1
            stop.wait(max(0, interval-(time.monotonic()-tick)))
    status.update(phase='stopped' if stop.is_set() else 'complete', elapsed_s=round(time.monotonic()-start, 2))
    if not status['captured_frames']:
        status['phase'] = 'error'
    atomic_json(output/'status.json', status)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=60)
    args = parser.parse_args()
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    collect(args.base, args.output, max(1, min(120, args.seconds)), stop=stop)


if __name__ == '__main__':
    main()
