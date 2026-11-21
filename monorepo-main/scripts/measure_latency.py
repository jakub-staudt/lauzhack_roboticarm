#!/usr/bin/env python3
"""Read-only HTTP/camera timing. Never claims control or sends POST requests."""
import argparse
import concurrent.futures
import datetime
import json
import threading
import time
import urllib.request


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    pos = (len(values) - 1) * q
    lo = int(pos)
    hi = min(lo + 1, len(values) - 1)
    return round(values[lo] + (values[hi] - values[lo]) * (pos - lo), 3)


def stats(values):
    return {"n": len(values), "min": min(values) if values else None,
            "p50": percentile(values, .5), "p95": percentile(values, .95),
            "max": max(values) if values else None}


def request_state(base):
    start = time.perf_counter()
    try:
        with urllib.request.urlopen(base + '/api/state', timeout=10) as response:
            headers_ms = (time.perf_counter() - start) * 1000
            payload = json.load(response)
            elapsed_ms = (time.perf_counter() - start) * 1000
            return {"elapsed_ms": round(elapsed_ms, 3), "headers_ms": round(headers_ms, 3),
                    "status": response.status, "connected": payload.get('connected'),
                    "loop_ok": payload.get('loop_ok'), "loop_hz": payload.get('loop_hz'),
                    "sequence": payload.get('sequence'),
                    "server_state_age_ms": round(payload['age_s'] * 1000, 3) if payload.get('age_s') is not None else None,
                    "cameras": {k: {"status": v.get('status'), "fps": v.get('fps'),
                                    "server_frame_age_ms": round(v['age_s'] * 1000, 3) if v.get('age_s') is not None else None}
                                for k, v in payload.get('cameras', {}).items()}}
    except Exception as exc:
        return {"elapsed_ms": round((time.perf_counter() - start) * 1000, 3), "error": str(exc)}


def camera(base, name, stop):
    start = time.perf_counter()
    result = {"name": name, "frame_receive_times_ms": [], "frame_sizes_bytes": []}
    try:
        with urllib.request.urlopen(base + '/cameras/' + name + '.mjpg', timeout=10) as response:
            result['status'] = response.status
            while not stop.is_set():
                line = response.readline()
                if not line:
                    break
                if line.strip() != b'--frame':
                    continue
                headers = {}
                while True:
                    line = response.readline()
                    if not line.strip():
                        break
                    key, value = line.decode().split(':', 1)
                    headers[key.lower()] = value.strip()
                size = int(headers['content-length'])
                data = response.read(size)
                if len(data) != size:
                    raise IOError('Incomplete JPEG frame')
                result['frame_receive_times_ms'].append(round((time.perf_counter() - start) * 1000, 3))
                result['frame_sizes_bytes'].append(size)
    except Exception as exc:
        result['error'] = str(exc)
    times = result['frame_receive_times_ms']
    result['first_complete_frame_ms'] = times[0] if times else None
    result['interarrival_ms'] = stats([round(b-a, 3) for a, b in zip(times, times[1:])])
    result['received_fps'] = round((len(times)-1)*1000/(times[-1]-times[0]), 3) if len(times)>1 else None
    result['duration_ms'] = round((time.perf_counter()-start)*1000, 3)
    return result


def phase(base, count, interval, streams):
    stop = threading.Event()
    started = datetime.datetime.now(datetime.timezone.utc).isoformat()
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        cameras = [executor.submit(camera, base, name, stop) for name in ['overhead', 'wrist']] if streams else []
        rows = []
        for i in range(count):
            rows.append(request_state(base))
            if i + 1 < count:
                time.sleep(interval)
        stop.set()
        frames = [f.result() for f in cameras]
    good = [r for r in rows if 'error' not in r]
    return {"started_utc": started, "streams_requested": streams, "samples": rows, "cameras": frames,
            "summary": {"successful": len(good), "errors": len(rows)-len(good),
                        "http_roundtrip_ms": stats([r['elapsed_ms'] for r in good]),
                        "server_state_age_ms": stats([r['server_state_age_ms'] for r in good if r['server_state_age_ms'] is not None]),
                        "responses_over_ui_1800ms": sum(r['elapsed_ms'] > 1800 for r in rows),
                        "responses_over_lease_1500ms": sum(r['elapsed_ms'] > 1500 for r in rows)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', action='append', required=True)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--interval', type=float, default=.3)
    parser.add_argument('--with-streams', action='store_true')
    parser.add_argument('--output')
    args = parser.parse_args()
    result = {"measured_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "method": "Sequential GET /api/state with fresh HTTP connections; read-only. Optional two continuous MJPEG streams. No POSTs, physical movement, browser rendering or sensor timestamps.",
              "samples_per_phase": args.samples, "interval_after_response_s": args.interval, "targets": {}}
    for base in args.base:
        phases = {"baseline": phase(base, args.samples, args.interval, False)}
        if args.with_streams:
            phases['two_cameras'] = phase(base, args.samples, args.interval, True)
        result['targets'][base] = phases
    output = json.dumps(result, indent=2) + '\n'
    if args.output:
        from pathlib import Path
        Path(args.output).write_text(output)
    else:
        print(output, end='')


if __name__ == '__main__':
    main()
