#!/usr/bin/env python3
"""Constructed analyzer boundary checks only; not replay or physical evidence."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parent
PATH = ROOT / 'analyze.py'
spec = importlib.util.spec_from_file_location('regression_analyzer_smoke', PATH)
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)

def require(condition, message):
    if not condition:
        raise RuntimeError(message)

call = dict(line=6, post_line=7, call_id=1, clock=42.999, clock_relative_s=42.999,
    wall=9.999, policy_clock=43.001, policy_wall=10.001, corrected_stamp=1601., raw_stamp=1501.,
    queued_line=5, model_epoch=2, session_epoch=2, content='original', visual_observations=1,
    legacy_retained=True, eligible=True, policy_counted=True, policy_success=True, policy_ignored=False,
    zero_pose_info_retained=False, zero_pose_increment=False, success_time_refreshed=True,
    failures_cleared=False, service_alive=True, input_fresh=True, model_usable=True,
    eligibility_reason='synthetic_smoke', policy_event_reason='synthetic_smoke',
    A_legacy_retained=True, B_measurement_commit=True, C_manager_success=True)
policy = a.predicate_call(call, 'C_manager_success', 0.)
# Both cutoffs intersect the same constructed call. Use production selection and count APIs.
pre = a.counts([call], 15., 43., policy_calls=[])
post = a.counts([], 43., 51., policy_calls=[policy])
require([pre[k] for k in a.PREDICATES] == [1, 1, 0], 'A/B before EOF must not import later policy credit')
require([post[k] for k in a.PREDICATES] == [0, 0, 1], 'C after EOF must retain its actual policy event')
require(post['C_manager_success_occupied_bins'] == [0], 'C integer window uses actual policy clock')
trace = [dict(event='vio', _line=6, wall=call['wall']),
         dict(event='visual_feedback', _line=7, wall=policy['wall'])]
first = {key: a.sustained(trace, {6: call}, 0., 0., 10., key)['first'] for key in a.PREDICATES}
require(first['A_legacy_retained'] is not None and first['B_measurement_commit'] is not None
        and first['C_manager_success'] is None, 'C sustained search must exclude post EOF event')
partial = a.counts([], 15., 123.27781414985657)
require(partial['denominator_fixed_second_bins'] == 109, 'Full attached interval denominator must be 109')
require(abs(partial['last_bin_sensor_seconds'] - .27781414985657) < 1e-12, 'Final partial bin duration changed')

origin = 1000.
def row(event, line, wall, relative, **extra):
    return dict(event=event, _line=line, _file='constructed_smoke', wall=wall, clock=origin+relative,
                model_epoch=2, session_epoch=2, health_state='PROBING', **extra)
trace = [dict(row('tick', 1, 1., 15.), model_epoch=1, session_epoch=1),
         row('model_epoch_staged', 2, 10.002, 51.002),
         row('applied', 3, 10.003, 51.003, last_raw_image=1500.),
         row('image_health_fresh', 4, 10.005, 51.005, event_raw=1501.),
         row('image_queued', 5, 10.006, 51.006, event_raw=1501.),
         row('vio', 6, 10.007, 51.007), row('visual_feedback', 7, 10.008, 51.008)]
service = [dict(event='started', wall=1., clock=origin+15., session_id='old'),
           dict(event='started', wall=10.001, clock=origin+51.001, session_id='new'),
           dict(event='sent', wall=10.004, clock=origin+51.004, session_id='new')]
runner = [dict(event='service_stop_request', wall=2., clock=origin+43.),
          dict(event='service_restart_request', wall=10., clock=origin+51.)]
call = dict(call, clock=origin+51.007, clock_relative_s=51.007, wall=10.007,
            policy_wall=10.008, policy_clock=origin+51.008)
result = a.fault_recovery('S', {}, trace, [call], runner, [], [], service, origin, 20.)
require(result['status'] == 'analyzed_until_natural_EOF', 'S recovery analysis rejected publish-before-sent ordering')
require(result['new_epoch_staged']['wall'] < result['first_new_session_sent']['wall'], 'Fixture must stage before sent log')
require(result['new_session_identity_distinct'] and result['staged_and_applied_epoch_equal'], 'New session/epoch association failed')
require(result['first_consumed_recovery_identity']['call_id'] == 1, 'Exact new epoch consumption was lost')

# A transient staged startup model can report offset 0. The last image must use
# its own exact frontend identity and applied 0.1 offset, not all-history values.
with tempfile.TemporaryDirectory(prefix='frontend_analysis_smoke_') as temporary:
    directory = Path(temporary)
    (directory / 'events.jsonl').write_text(json.dumps(dict(event='input', topic='/sensor/imu',
        stamp_ns=1550000000, clock=2.)) + '\n')
    pixel = dict(stamp_ns=1501000000, raw_stamp=1.501, clock=2., wall=20., content='original')
    endpoint_trace = [dict(event='model_epoch_staged', model_usable=1, offset=0., wall=1., clock=1.),
        dict(event='image_queued', model_usable=1, offset=.1, event_raw=1.501,
             event_corrected=1.601, wall=20.001, clock=2., _line=2)]
    endpoint = a.input_endpoints(directory, [pixel], endpoint_trace, 0.)
require(endpoint['observed_usable_model_offsets'] == [0., .1], 'Fixture must contain startup offset transition')
require(endpoint['last_image_actual_offsets'] == [.1], 'Last-image offset binding was lost')
require(endpoint['last_image_corrected_minus_last_imu_ns'] == 51000000, 'Startup offset contaminated tail comparison')
print(json.dumps(dict(scope='Constructed analyzer checks only, not new replay or physical accuracy evidence',
    analyzer_sha256=hashlib.sha256(PATH.read_bytes()).hexdigest(), passed=True,
    cases={'cross_EOF_and_integer_window': {'before': [pre[k] for k in a.PREDICATES], 'after': [post[k] for k in a.PREDICATES]},
           'partial_bin': {'denominator': partial['denominator_fixed_second_bins'], 'last_bin_s': partial['last_bin_sensor_seconds']},
           'S_publish_before_sent': {'started_wall': service[1]['wall'], 'staged_wall': result['new_epoch_staged']['wall'],
                                    'sent_wall': result['first_new_session_sent']['wall'], 'associated_epoch': result['new_epoch_applied']['model_epoch']},
           'tail_offset_identity': {'all_history_offsets': endpoint['observed_usable_model_offsets'],
                                    'last_image_offsets': endpoint['last_image_actual_offsets'],
                                    'corrected_image_minus_imu_ns': endpoint['last_image_corrected_minus_last_imu_ns']}}), indent=2))
