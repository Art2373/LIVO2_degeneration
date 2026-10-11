"""Unchanged historical event predicates; no estimator dependencies."""
import math
import numpy as np

CACHES = ('pending_vio', 'pending_image', 'image_queue', 'visual_voxels', 'warp_cache',
          'reference_frame', 'rgb_cached', 'color_cache', 'visual_observations')

HEALTH = ('health_state', 'service_alive', 'input_fresh', 'model_usable', 'model_epoch',
          'session_epoch', 'policy_reason', 'probe_attempt', 'probe_calls',
          'consecutive_successes', 'consecutive_failures', 'actual_update_time', 'cache_clear_count',
          'health_previous', 'probe_changes_filter', 'management_enabled', 'state_finite')

def finite(value):
    return isinstance(value, (int, float, np.integer, np.floating)) and math.isfinite(float(value))

def plain(value):
    if isinstance(value, np.ndarray):
        return plain(value.tolist())
    if isinstance(value, dict):
        return {str(key): plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if isinstance(value, (np.integer, np.bool_)):
        return value.item()
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(float(value)) else None
    return value

def distribution(values):
    values = np.asarray([float(value) for value in values if finite(value)])
    if not len(values):
        return dict(n=0)
    return dict(n=len(values), min=float(values.min()), median=float(np.median(values)),
                p95=float(np.percentile(values, 95)), max=float(values.max()))

def actual_vio(row):
    counter_evidence = (row.get('event') == 'vio' and row.get('vio_steps', 0) > row.get('vio_reverted', 0)
                        and row.get('vio_pixels', 0) > 0)
    return counter_evidence and bool(row['vio_retained_finite']) if 'vio_retained_finite' in row else counter_evidence

def point(row):
    if row is None:
        return None
    keys = ['event', 'wall', 'clock', 'state_time', 'active', 'source', 'session_id', 'pid',
            'revision', 'vio_steps', 'vio_reverted', 'vio_pixels', 'visual_observations',
            'vio_image_raw_time', 'vio_image_corrected_time', 'message_stamp', 'command_id',
            'vio_retained_finite', 'vio_nonfinite', 'last_raw_image', 'last_corrected_image',
            'last_reason', 'event_raw', 'event_corrected']
    return {key: row[key] for key in keys + list(HEALTH) + list(CACHES) if key in row}

def difference(later, earlier):
    if later is None or earlier is None:
        return None
    return {name + '_s': later[key] - earlier[key] if finite(later.get(key)) and finite(earlier.get(key)) else None
            for name, key in [('wall', 'wall'), ('sensor_clock', 'clock')]}

def cache_state(row):
    values = {key: row[key] for key in CACHES if key in row}
    return dict(event=point(row), fields=values,
                all_recorded_fields_empty=all(value == 0 for value in values.values()) if values else None,
                complete_field_coverage=len(values) == len(CACHES))
