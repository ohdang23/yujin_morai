"""ROS-independent trial recording, fingerprints, packaging and summaries."""
import collections
import hashlib
import json
import math
from pathlib import Path
import tarfile

def apply_profile(config, profile):
    """Recording baseline preserves all supplied driving parameters."""
    if profile != 'baseline':
        raise ValueError('Only the baseline recording profile is supported: '+profile)
    return dict(config)


def sha256(path):
    h = hashlib.sha256()
    with open(str(path), 'rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''):
            h.update(b)
    return h.hexdigest()


def package_trial(directory):
    directory = Path(directory).resolve()
    output = directory.with_name(directory.name+'.tar.gz')
    with tarfile.open(str(output), 'w:gz') as archive:
        archive.add(str(directory), arcname=directory.name)
    return output


def quantiles(values):
    values = sorted(float(v) for v in values if isinstance(v, (int,float)) and math.isfinite(v))
    if not values:
        return None
    return dict(n=len(values), min=values[0], p50=values[len(values)//2],
                p95=values[min(len(values)-1, int(len(values)*.95))], max=values[-1])


def summarize(directory):
    directory = Path(directory)
    rows, broken = [], 0
    for path in sorted(directory.glob('control_*.jsonl')):
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                try:
                    rows.append(json.loads(line))
                except (ValueError, TypeError):
                    broken += 1
    clock = lambda r: r.get('log_monotonic_s', r.get('timestamp', 0.))
    rows.sort(key=clock)
    reason_samples = collections.Counter()
    reason_seconds = collections.Counter()
    transitions, previous = [], None
    slow_only = total_prediction = 0
    for i,r in enumerate(rows):
        reason = r.get('decision_reason_detail',r.get('decision_reason','unknown'))
        reason_samples[reason] += 1
        if i+1 < len(rows):
            dt = clock(rows[i+1])-clock(r)
            if 0 <= dt <= 1.:
                reason_seconds[reason] += dt
        label = (r.get('state'), reason, r.get('stop_required'), r.get('emergency'))
        if label != previous:
            transitions.append(dict(timestamp=r.get('timestamp'), elapsed_s=clock(r)-clock(rows[0]),
                state=r.get('state'), reason=reason, stop=r.get('stop_required'),
                emergency=r.get('emergency'), blocker_sources=r.get('blocker_sources',[])))
            previous = label
        hits = r.get('prediction_profile_hits', [])
        if hits:
            total_prediction += 1
            if all(h['ego_profile']=='slow' for h in hits):
                slow_only += 1
    launches = []
    for i, r in enumerate(rows):
        if r.get('decision_reason') != 'safe_gap_confirmed':
            continue
        # Several driver ticks may carry the same decision frame.
        stamp = r.get('driver_status_stamp',r.get('timestamp'))
        if launches and launches[-1]['decision_stamp'] == stamp:
            continue
        event = dict(decision_stamp=stamp, recorded_at=r.get('timestamp'),
                     ego_speed_at_permission=r.get('driver_ego_speed',r.get('ego_speed')))
        subsequent = []
        for other in rows[i:]:
            if clock(other)-clock(r)>15.:
                break
            if other.get('state') not in ('COMMIT','FOLLOW','EXIT'):
                break
            subsequent.append(other)
        conditions = {
            'throttle_delay_s': lambda x: (x.get('accel_cmd') or 0)>0.01 and (x.get('brake_cmd') or 0)==0,
            'motion_delay_s': lambda x: (x.get('driver_ego_speed') or 0)>=0.2,
            'speed_2mps_delay_s': lambda x: (x.get('driver_ego_speed') or 0)>=2.,
        }
        for key, test in conditions.items():
            hit = next((x for x in subsequent if test(x)), None)
            event[key] = None if hit is None else clock(hit)-clock(r)
        launches.append(event)
    metrics = {}
    for key in ('decision_compute_ms','gate_tick_period_s','driver_tick_period_s',
                'obstacle_source_period_s','obstacle_age_s','driver_status_age_s'):
        metrics[key] = quantiles([r.get(key) for r in rows])
    result = dict(control_samples=len(rows), malformed_json_lines=broken,
                  observed_duration_s=clock(rows[-1])-clock(rows[0]) if rows else 0,
                  reason_samples=dict(reason_samples), reason_seconds=dict(reason_seconds),
                  trajectory_blocked_samples=total_prediction,
                  slow_profile_only_blocked_samples=slow_only,
                  launches=launches, metrics=metrics, transitions=transitions,
                  interpretation='Sampled diagnostics only. Not a collision-free validation; delays measured from first recorded permission sample.')
    (directory/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result
