#!/usr/bin/env python3
"""Read-only natural-EOF regression analysis for H/R/B/S; no ROS or replay.

A historical retained, B finite measurement commit, and C actual manager credit
remain separate. Fixed sensor windows exclude every row at/after observed EOF
wall time; the following three wall seconds are a separate drain observation.
"""
import argparse
from collections import Counter
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
import metrics as old
import queue_audit as base
import summary_helpers as prior
CASES = ('H', 'R', 'B', 'S')
PREDICATES = ('A_legacy_retained', 'B_measurement_commit', 'C_manager_success')
GUARDS = {
    'duplicate_call_id': ('duplicate_or_old_call_id',),
    'feedback_epoch_mismatch': ('result_model_epoch_mismatch',),
    'cross_epoch_or_time_reset': ('result_crossed_epoch_or_time_reset',),
    'stale_duplicate_or_preprobe_stamp': ('duplicate_nonfinite_or_preprobe_vio_result',),
    'unscheduled_result': ('vio_not_executed',),
    'prerequisite_loss': ('result_after_prerequisite_loss',),
    'unpermitted_result': ('result_was_not_permitted_before_call',),
}
INPUTS = {}


def read(path):
    path = Path(path)
    content = path.read_bytes()
    INPUTS[str(path)] = hashlib.sha256(content).hexdigest()
    return content.decode()


def document(path):
    return json.loads(read(path))


def rows(path):
    if not Path(path).exists():
        return []
    return [dict(json.loads(line), _line=index, _file=str(path))
            for index, line in enumerate(read(path).splitlines(), 1) if line.strip()]


def write(path, value):
    with path.open('x') as stream:
        json.dump(old.plain(value), stream, indent=2, allow_nan=False)


def csv_write(path, values):
    columns = list(dict.fromkeys(key for value in values for key in value))
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns or ['empty'])
        writer.writeheader()
        for value in values:
            writer.writerow({key: json.dumps(old.plain(item), ensure_ascii=False)
                             if isinstance(item, (list, tuple, dict)) else item
                             for key, item in value.items()})


def point(row, origin):
    if row is None:
        return None
    value = prior.point(row, origin)
    for key in ('_file', 'session_id', 'session_epoch', 'model_epoch', 'revision', 'pid',
                'feedback_model_epoch', 'feedback_sensor_stamp', 'active', 'event_raw',
                'event_corrected', 'probe_attempt', 'probe_calls', 'sha256', 'encoding',
                'width', 'height', 'source', 'image_stamp'):
        if key in row:
            value[key] = row[key]
    return value


def first(values, events):
    return next((value for value in values if value.get('event') in events), None)


def check_feedback(trace, mode):
    """Same-call audit independent of fixed durations or fault content.

    Counter effects follow the established feedback audit; actual guard branches
    have explicit exercised/not-covered labels rather than vacuous validation.
    """
    before = [row for row in trace if row['event'] == 'vio']
    after = [row for row in trace if row['event'] == 'visual_feedback']
    prem = {row['feedback_call_id']: row for row in before}
    postm = {row['feedback_call_id']: row for row in after}
    violations, checks = [], []

    def require(condition, reason, ident=None):
        if not condition:
            violations.append(dict(call_id=ident, reason=reason))

    require(len(prem) == len(before), 'duplicate_raw_call_id')
    require(len(postm) == len(after), 'duplicate_post_call_id')
    require(set(prem) == set(postm), 'raw_post_identity_set_mismatch')
    ids = [row['feedback_call_id'] for row in before]
    require(all(b > a for a, b in zip(ids, ids[1:])), 'call_ids_not_monotonic')
    for ident in sorted(set(prem) & set(postm)):
        pre, post = prem[ident], postm[ident]
        counted, success, ignored = (bool(post[key]) for key in
            ('policy_result_counted', 'policy_result_success', 'policy_event_ignored'))
        same_epoch = pre['model_epoch'] == post['model_epoch'] == pre['feedback_model_epoch']
        same_probe = pre['probe_attempt'] == post['probe_attempt']
        reset = not same_epoch or not same_probe
        expected = bool(pre['feedback_eligible']) if mode == 'measurement_commit_v1' else bool(pre['vio_retained_finite'])
        df = post['consecutive_failures'] - pre['consecutive_failures']
        ds = post['consecutive_successes'] - pre['consecutive_successes']
        refreshed = post['policy_last_success_time'] > pre['policy_last_success_time']
        require(post['_line'] > pre['_line'], 'post_pre_order', ident)
        for key in ('feedback_sensor_stamp', 'feedback_model_epoch', 'feedback_eligible'):
            require(pre[key] == post[key], 'changed_' + key, ident)
        require(pre['policy_result_counted'] == 0 and pre['policy_result_success'] == 0,
                'credit_not_reset_before_call', ident)
        require(not success or counted, 'success_without_count', ident)
        require(counted != ignored, 'count_ignore_not_complementary', ident)
        if post['policy_event_reason'] in {reason for reasons in GUARDS.values() for reason in reasons}:
            require(ignored and not counted and not success, 'observed_guard_did_not_reject_credit', ident)
        if pre['feedback_eligible']:
            require(bool(old.actual_vio(pre)) and pre['feedback_executed'] and
                    pre['feedback_completed'] and pre['feedback_final_finite'] and
                    not pre['feedback_nonfinite_attempt'] and pre['feedback_surviving_accepts'] > 0 and
                    pre['feedback_surviving_pose_information'] and pre['feedback_covariance_committed'] and
                    pre['feedback_covariance_finite'], 'eligible_without_finite_measurement_commit', ident)
        if counted:
            require(same_epoch and same_probe, 'counted_cross_epoch_or_probe', ident)
            require(pre['health_state'] in ('PROBING', 'FUSING'), 'counted_from_unpermitted_state', ident)
            require(success == expected, 'credit_not_selected_feedback_semantics', ident)
            if success:
                require(post['policy_last_success_time'] == pre['feedback_sensor_stamp'],
                        'success_time_not_actual_stamp', ident)
                require(post['consecutive_failures'] == 0 and ds == 1, 'wrong_success_counter_effect', ident)
            else:
                require(df == 1 and post['consecutive_successes'] == 0, 'wrong_failure_counter_effect', ident)
                require(post['policy_last_success_time'] == pre['policy_last_success_time'],
                        'non_success_changed_success_time', ident)
            require(post['probe_calls'] - pre['probe_calls'] == (1 if pre['health_state'] == 'PROBING' else 0),
                    'wrong_probe_call_effect', ident)
        else:
            require(not success, 'ignored_result_credited', ident)
            if not reset:
                require(df == 0 and ds == 0, 'ignored_result_changed_streak_without_reset', ident)
                require(post['policy_last_success_time'] == pre['policy_last_success_time'],
                        'ignored_result_changed_success_time_without_reset', ident)
                require(post['probe_calls'] == pre['probe_calls'], 'ignored_result_changed_probe_count', ident)
        if mode == 'measurement_commit_v1' and not pre['feedback_surviving_pose_information']:
            require(not pre['feedback_eligible'] and not success, 'zero_information_eligible_or_credited', ident)
            if counted:
                require(not refreshed and df == 1, 'zero_information_wrong_counter_effect', ident)
        if not pre['feedback_executed'] or not same_epoch:
            require(not counted, 'unscheduled_or_cross_epoch_counted', ident)
        checks.append(dict(call_id=ident, pre_line=pre['_line'], post_line=post['_line'],
            raw_eligible=bool(pre['feedback_eligible']), actual_counted=counted,
            actual_success=success, ignored=ignored, event_reason=post['policy_event_reason'],
            same_epoch=same_epoch, same_probe=same_probe, poll_reset_possible=reset,
            failures_before=pre['consecutive_failures'], failures_after=post['consecutive_failures'],
            successes_before=pre['consecutive_successes'], successes_after=post['consecutive_successes'],
            success_time_before=pre['policy_last_success_time'], success_time_after=post['policy_last_success_time'],
            success_time_refreshed=refreshed))
    coverage = {}
    for name, reasons in GUARDS.items():
        covered = [row for row in checks if row['event_reason'] in reasons]
        coverage[name] = dict(status='covered' if covered else 'not_covered', count=len(covered),
            call_ids=[row['call_id'] for row in covered],
            all_observed_ignored_without_credit=all(row['ignored'] and not row['actual_counted'] and
                                                  not row['actual_success'] for row in covered) if covered else None)
    ignored_rows = [row for row in checks if row['ignored']]
    coverage['any_ignored_result'] = dict(status='covered' if ignored_rows else 'not_covered', count=len(ignored_rows))
    epoch_rows = [row for row in trace if row['event'] == 'model_epoch_staged']
    coverage['model_epoch_staging'] = dict(status='covered' if epoch_rows else 'not_covered',
        count=len(epoch_rows), lines=[row['_line'] for row in epoch_rows],
        note='Staging an epoch does not exercise rejection of an in-flight mismatched result.')
    return dict(passed=not violations, violations=violations, calls=len(checks), coverage=coverage,
                ignored_reasons=dict(Counter(row['event_reason'] for row in ignored_rows))), checks, postm


def build_calls(case, trace, pixels, consumed, postm, origin, eof_wall):
    identities = {row['line']: row for row in consumed}
    delivery = {row['raw_stamp']: row for row in pixels}
    if len(delivery) != len(pixels):
        raise ValueError('Duplicate delivered image identity')
    result = []
    for row in trace:
        if row['event'] != 'vio':
            continue
        ident, post = identities[row['_line']], postm[row['feedback_call_id']]
        pixel = delivery[ident['raw_stamp']]
        if not pixel['known_input'] or pixel['content'] not in ('black', 'original'):
            raise ValueError('Unknown delivered pixel source')
        a, b = bool(old.actual_vio(row)), bool(row['feedback_eligible'])
        counted = bool(post['policy_result_counted'])
        credit = counted and bool(post['policy_result_success']) and not bool(post['policy_event_ignored'])
        call = dict(case=case, line=row['_line'], post_line=post['_line'], call_id=row['feedback_call_id'],
            clock=row['clock'], clock_relative_s=row['clock'] - origin, wall=row['wall'],
            after_eof=row['wall'] >= eof_wall, policy_wall=post['wall'], policy_clock=post['clock'],
            policy_after_eof=post['wall'] >= eof_wall, raw_stamp=ident['raw_stamp'], corrected_stamp=ident['sensor_stamp'],
            queued_line=ident['queued_line'], queued_wall=ident['queued_wall'], pixel_line=pixel['_line'],
            pixel_stamp_ns=pixel['stamp_ns'], pixel_sha256=pixel['sha256'], content=pixel['content'],
            A_legacy_retained=a, B_measurement_commit=b, C_manager_success=credit,
            legacy_retained=a, eligible=b, policy_counted=counted, policy_success=credit,
            policy_ignored=bool(post['policy_event_ignored']), eligibility_reason=row['feedback_eligibility_reason'],
            policy_event_reason=post['policy_event_reason'], zero_pose_info_retained=a and not row['feedback_surviving_pose_information'],
            zero_pose_increment=row['vio_translation_increment'] == 0 and row['vio_rotation_increment_rad'] == 0,
            success_time_refreshed=post['policy_last_success_time'] > row['policy_last_success_time'],
            failures_cleared=row['consecutive_failures'] > 0 and post['consecutive_failures'] == 0,
            model_epoch=row['model_epoch'], session_epoch=row['session_epoch'], feedback_epoch=row['feedback_model_epoch'],
            service_alive=bool(row['service_alive']), input_fresh=bool(row['input_fresh']),
            model_usable=bool(row['model_usable']), visual_observations=row['visual_observations'],
            visual_voxels=row['visual_voxels'], state_before=row['health_state'], state_after=post['health_state'],
            survived_accepts=row['feedback_surviving_accepts'], surviving_pose_info=bool(row['feedback_surviving_pose_information']),
            covariance_basis_survives=bool(row['feedback_covariance_basis_survives']),
            covariance_committed=bool(row['feedback_covariance_committed']), covariance_finite=bool(row['feedback_covariance_finite']),
            covariance_max_delta=row['vio_covariance_max_delta'], final_finite=bool(row['feedback_final_finite']),
            nonfinite_attempt=bool(row['feedback_nonfinite_attempt']), failures_before=row['consecutive_failures'],
            failures_after=post['consecutive_failures'], successes_before=row['consecutive_successes'],
            successes_after=post['consecutive_successes'], probe_attempt=row['probe_attempt'],
            source_line=dict(file=row['_file'], line=row['_line']),
            pixel_identity=dict(file=pixel['_file'], line=pixel['_line'], stamp_ns=pixel['stamp_ns'], sha256=pixel['sha256']))
        result.append(call)
    return result


def predicate_call(call, predicate, origin):
    if predicate != 'C_manager_success':
        return call
    return dict(call, solver_wall=call['wall'], solver_clock=call['clock'],
                wall=call['policy_wall'], clock=call['policy_clock'],
                clock_relative_s=call['policy_clock'] - origin, qualifying_event='visual_feedback')


def counts(calls, lo=None, hi=None, policy_calls=None):
    value = prior.counts(calls)
    value.update({key: sum(bool(row[key]) for row in calls) for key in PREDICATES})
    if policy_calls is not None:
        value.update(C_manager_success=sum(row['C_manager_success'] for row in policy_calls),
            policy_counted=sum(row['policy_counted'] for row in policy_calls),
            policy_success=sum(row['policy_success'] for row in policy_calls),
            policy_non_success=sum(row['policy_counted'] and not row['policy_success'] for row in policy_calls),
            policy_ignored=sum(row['policy_ignored'] for row in policy_calls),
            policy_events=len(policy_calls))
    if lo is not None:
        denominator = math.ceil(hi - lo)
        value.update(begin_s=lo, end_s=hi, denominator_sensor_seconds=hi - lo,
            denominator_fixed_second_bins=denominator,
            last_bin_sensor_seconds=hi - lo - math.floor(hi - lo) or 1.,
            coverage_definition='Number of occupied registered 1-second bins / ceil(hi-lo); last partial bin counts as one and its length is listed.')
        for key in PREDICATES:
            selected = policy_calls if key == 'C_manager_success' and policy_calls is not None else calls
            occupied = sorted({math.floor(row['clock_relative_s'] - lo) for row in selected if row[key]})
            value[key + '_occupied_bins'] = occupied
            value[key + '_coverage'] = len(occupied) / denominator if denominator else None
    return value


def sustained(trace, calls_by_line, origin, begin_wall, end_wall, predicate, admissible=None):
    run, first_call = [], None
    event = 'visual_feedback' if predicate == 'C_manager_success' else 'vio'
    lookup = ({call['post_line']: call for call in calls_by_line.values()}
              if predicate == 'C_manager_success' else calls_by_line)
    for row in trace:
        if not begin_wall <= row['wall'] < end_wall:
            continue
        if row['event'] in ('withdrawn', 'applied', 'model_epoch_staged'):
            run = []
        if row['event'] != event:
            continue
        call = predicate_call(lookup[row['_line']], predicate, origin)
        if not call[predicate] or (admissible is not None and not admissible(call)):
            run = []
            continue
        if first_call is None:
            first_call = call
        if run and call['corrected_stamp'] <= run[-1]['corrected_stamp']:
            run = []
        run.append(call)
        if len(run) >= 3 and call['corrected_stamp'] - run[0]['corrected_stamp'] >= .2:
            return dict(status='sustained', first=first_call, achieved=call, count=len(run),
                        corrected_image_span_s=call['corrected_stamp'] - run[0]['corrected_stamp'],
                        calls=run, required_count=3, required_span_sensor_s=.2)
    return dict(status='not_sustained_before_boundary', first=first_call, achieved=None,
                trailing_count=len(run), trailing_calls=run, required_count=3, required_span_sensor_s=.2)


def snapshot(row, origin):
    if row is None:
        return None
    value = point(row, origin)
    keys = ('image_queue', 'queue_first_stamp', 'queue_last_stamp', 'pending_vio', 'pending_image',
            'raw_queue', 'imu_queue', 'latest_lidar_end', 'latest_imu', 'state_time',
            'last_raw_image', 'last_corrected_image', 'lio_calls', 'vio_calls', 'visual_voxels',
            'warp_cache', 'reference_frame', 'last_lidar_arrival_wall', 'last_imu_arrival_wall')
    value.update({key: row.get(key) for key in keys})
    value['unavailable_fields'] = [key for key in keys if key not in row]
    return value


def fault_recovery(case, manifest, trace, calls, runner, gate, pixels, services, origin, eof_wall):
    byline = {call['line']: call for call in calls}
    if case == 'H':
        return dict(applicable=False, reason='healthy_no_registered_fault')
    starts = {'R': ('rgb_drop_command',), 'B': ('occlusion_black_command',),
              'S': ('service_stop_request', 'service_stop', 'service_stop_command', 'reference_service_stop')}
    restores = {'R': ('rgb_restore_command',), 'B': ('occlusion_restore_command',),
                'S': ('service_restart_request', 'service_restart', 'service_restart_command', 'reference_service_restart')}
    fault, restore = first(runner, starts[case]), first(runner, restores[case])
    if fault is None or restore is None:
        return dict(applicable=True, status='missing_fault_or_restore_command',
                    commands=[point(row, origin) for row in runner if 'service' in row['event']])
    result = dict(applicable=True, fault=point(fault, origin), restore=point(restore, origin),
        prefault={key: sustained(trace, byline, origin, origin_wall(trace, origin + 15), fault['wall'], key)
                  for key in PREDICATES})
    result['exposure_by_predicate'] = {key: value['status'] == 'sustained'
                                      for key, value in result['prefault'].items()}
    result['B_and_C_prefault_exposed'] = all(result['exposure_by_predicate'][key]
                                            for key in ('B_measurement_commit', 'C_manager_success'))
    result['exposure_note'] = 'Computed from actual sustained B/C calls; runner legacy retained_seen precheck is not sufficient exposure evidence.'
    anchors = {'restore_command': restore}
    if case in ('R', 'B'):
        after_gate = next((row for row in gate if row['event'] == 'first_image_after_command'
                          and row.get('command_id', '').endswith('_restore') and row['wall'] >= restore['wall']), None)
        if after_gate is None:
            result.update(status='no_observed_restored_gate_image')
            return result
        restored_pixel = next((row for row in pixels if row['raw_stamp'] == after_gate['message_stamp']), None)
        if restored_pixel is None or not restored_pixel['original_payload']:
            result.update(status='no_exact_original_pixel_witness', restored_gate=point(after_gate, origin))
            return result
        anchors['first_restored_original_pixel'] = restored_pixel
        result.update(restored_gate=point(after_gate, origin), restored_pixel=point(restored_pixel, origin))
        admissible = lambda call: call['content'] == 'original' and call['raw_stamp'] >= restored_pixel['raw_stamp']
        admissible_image = lambda row: row['event_raw'] >= restored_pixel['raw_stamp']
        if case == 'R':
            affected = next((row for row in gate if row['event'] == 'first_image_after_command'
                            and row.get('command_id', '').endswith('_drop')), None)
            illegal = [call for call in calls if affected and
                       affected['message_stamp'] <= call['raw_stamp'] < restored_pixel['raw_stamp']]
            result['drop_identity_check'] = dict(first_dropped=point(affected, origin),
                illegal_consumptions=illegal, passed=affected is not None and not illegal,
                legal_prefault_tail_calls=[call for call in calls if affected and
                    fault['wall'] <= call['wall'] < restore['wall'] and call['raw_stamp'] < affected['message_stamp']])
    else:
        starts_service = [row for row in services if row['event'] == 'started']
        new_session = next((row for row in starts_service if row['wall'] >= restore['wall']), None)
        if new_session is None:
            result.update(status='new_service_session_not_observed')
            return result
        sent = next((row for row in services if row['event'] == 'sent' and row['session_id'] == new_session['session_id']), None)
        previous_epoch = max((row['model_epoch'] for row in trace if row['wall'] < new_session['wall']), default=0)
        staged = next((row for row in trace if row['event'] == 'model_epoch_staged'
                       and row['wall'] >= new_session['wall'] and row['model_epoch'] > previous_epoch), None)
        applied = next((row for row in trace if staged and row['event'] == 'applied' and
                        row['_line'] > staged['_line'] and row['model_epoch'] == staged['model_epoch']), None)
        result.update(new_session=point(new_session, origin), first_new_session_sent=point(sent, origin),
                      new_epoch_staged=point(staged, origin), new_epoch_applied=point(applied, origin),
                      session_epoch_binding='Unique newly started service process/session plus the first incremented model epoch after startup; frontend does not log the session string. publish precedes the service sent log, so sent/received wall ordering is not required.')
        if applied is None:
            result.update(status='new_session_not_applied_before_observation_end')
            return result
        prior_sessions = {row['session_id'] for row in starts_service if row['wall'] < new_session['wall']}
        result['new_session_identity_distinct'] = new_session['session_id'] not in prior_sessions
        result['staged_and_applied_epoch_equal'] = staged['model_epoch'] == applied['model_epoch']
        anchors.update(new_session_started=new_session, new_epoch_applied=applied)
        if sent:
            anchors['first_new_session_sent'] = sent
        admissible = lambda call: (call['content'] == 'original' and call['model_epoch'] == applied['model_epoch']
            and call['session_epoch'] == applied['session_epoch'] and call['queued_line'] > applied['_line']
            and call['raw_stamp'] > applied['last_raw_image'])
        admissible_image = lambda row: (row['_line'] > applied['_line'] and
            row['model_epoch'] == applied['model_epoch'] and row['event_raw'] > applied['last_raw_image'])
    legal = next((row for row in trace if row['event'] == 'image_health_fresh' and admissible_image(row)), None)
    admitted = next((row for row in trace if row['event'] == 'image_queued' and admissible_image(row)), None)
    result['first_lawful_recovery_image'] = point(legal, origin)
    result['first_admitted_recovery_image'] = point(admitted, origin)
    first_consumed = next((call for call in calls if call['wall'] < eof_wall and admissible(call)), None)
    result['first_consumed_recovery_identity'] = first_consumed
    result['anchors'] = {name: point(row, origin) for name, row in anchors.items()}
    result['predicates'] = {}
    for key in PREDICATES:
        found = sustained(trace, byline, origin, restore['wall'], eof_wall, key, admissible)
        found['prefault_exposed'] = result['exposure_by_predicate'][key]
        found['delays'] = {name: dict(first=old.difference(found['first'], anchor),
                                      sustained=old.difference(found['achieved'], anchor))
                           for name, anchor in anchors.items()}
        result['predicates'][key] = found
    result['first_FUSING_after_restore'] = point(next((row for row in trace if
        restore['wall'] <= row['wall'] < eof_wall and row['event'] == 'health_transition' and
        row['health_state'] == 'FUSING'), None), origin)
    result['status'] = 'analyzed_until_natural_EOF'
    return result


def origin_wall(trace, stamp):
    return next((row['wall'] for row in trace if row['clock'] >= stamp), math.inf)


def withdrawals(trace, calls, origin, eof_wall, manifest):
    byline = {call['line']: call for call in calls}
    events = [row for row in trace if row['event'] == 'withdrawn' and row['wall'] < eof_wall
              and row['clock'] >= origin + 15]
    output = []
    for index, row in enumerate(events):
        following = events[index + 1] if index + 1 < len(events) else None
        end_wall = following['wall'] if following else eof_wall
        relative = row['clock'] - origin
        context = ('registered_fault_window' if manifest.get('events') and 43 <= relative < 51
                   else 'outside_registered_fault_window')
        episode = dict(withdrawal=point(row, origin), context=context,
            note='Observed withdrawal; not labelled a false positive from image content alone.',
            next_withdrawal=point(following, origin), boundary='next_withdrawal' if following else 'natural_EOF',
            immediate={}, eventual_until_EOF={})
        for key in PREDICATES:
            now = sustained(trace, byline, origin, row['wall'], end_wall, key)
            now['delay_from_withdrawal'] = old.difference(now['achieved'], row)
            episode['immediate'][key] = now
            eventual = sustained(trace, byline, origin, row['wall'], eof_wall, key)
            eventual['delay_from_withdrawal'] = old.difference(eventual['achieved'], row)
            episode['eventual_until_EOF'][key] = eventual
        output.append(episode)
    return output


def long_gaps(trace, calls, origin, duration, eof_wall):
    result = {}
    for predicate in PREDICATES:
        candidates = [predicate_call(call, predicate, origin) for call in calls]
        good = [call for call in candidates if 15 <= call['clock_relative_s'] < duration
                and call['wall'] < eof_wall and call[predicate]]
        edges = [dict(clock_relative_s=15., wall=origin_wall(trace, origin + 15), call_id=None)] + good + [
            dict(clock_relative_s=duration, wall=eof_wall, call_id=None)]
        gaps = []
        for left, right in zip(edges, edges[1:]):
            if right['clock_relative_s'] - left['clock_relative_s'] <= 1.:
                continue
            inside = [row for row in trace if left['wall'] <= row['wall'] <= right['wall']]
            gaps.append(dict(begin_sensor_s=left['clock_relative_s'], end_sensor_s=right['clock_relative_s'],
                duration_sensor_s=right['clock_relative_s'] - left['clock_relative_s'],
                duration_wall_s=right['wall'] - left['wall'], left_call_id=left['call_id'], right_call_id=right['call_id'],
                includes_fixed_interval_edge=left['call_id'] is None or right['call_id'] is None,
                event_counts=dict(Counter(row['event'] for row in inside)),
                reason_counter_deltas={key: inside[-1]['reason_counts'].get(key, 0) - inside[0]['reason_counts'].get(key, 0)
                    for key in ('wait_reorder', 'wait_imu', 'wait_imu_coverage', 'image_inactive')}
                    if inside else {}))
        result[predicate] = gaps
    return result


def input_endpoints(directory, pixels, trace, origin):
    path = directory / 'events.jsonl'
    latest, counts_by_topic = {}, Counter()
    if path.exists():
        for index, line in enumerate(read(path).splitlines(), 1):
            if not line.strip():
                continue
            row = json.loads(line)
            if row.get('event') != 'input' or 'stamp_ns' not in row:
                continue
            topic = row['topic']
            counts_by_topic[topic] += 1
            if topic not in latest or row['stamp_ns'] > latest[topic]['stamp_ns']:
                latest[topic] = dict(file=str(path), line=index, stamp_ns=row['stamp_ns'], clock=row.get('clock'))
    last_pixel = pixels[-1] if pixels else None
    offsets = sorted({row['offset'] for row in trace if row.get('model_usable')})
    last_image_rows = [row for row in trace if last_pixel and
        row['event'] in ('image_health_fresh', 'image_decoded', 'image_queued') and
        row['event_raw'] == last_pixel['raw_stamp'] and row.get('model_usable')]
    last_image_offsets = sorted({row['offset'] for row in last_image_rows})
    imu = next((value for topic, value in latest.items() if topic.endswith('/imu')), None)
    difference_ns = None
    if last_pixel and imu and len(last_image_offsets) == 1:
        difference_ns = last_pixel['stamp_ns'] + round(last_image_offsets[0] * 1e9) - imu['stamp_ns']
    return dict(last_monitored_header_stamp_by_topic=latest, monitored_input_counts=dict(counts_by_topic),
        last_delivered_pixel=point(last_pixel, origin), observed_usable_model_offsets=offsets,
        last_image_actual_offsets=last_image_offsets,
        last_image_frontend_identity=[dict(point(row, origin), offset=row['offset']) for row in last_image_rows],
        last_image_corrected_minus_last_imu_ns=difference_ns,
        last_corrected_image_exceeds_last_imu=difference_ns > 0 if difference_ns is not None else None,
        limitation='LiDAR message header is not the last point timestamp. latest_lidar_end in mapper snapshots is available, but exact point-level endpoint provenance is not reconstructed here. Positive image-minus-IMU confirms unavailable coverage at that cut; it does not imply every residual queue entry needs that cut.')


def analyze_case(directory, out):
    manifest = document(directory / 'manifest.json')
    if not manifest['completed'] or manifest['feedback_mode'] != 'measurement_commit_v1' or manifest['recovery_schedule'] != 'burst':
        raise ValueError('Requires completed managed measurement_commit_v1 / burst run')
    if manifest['policy'] != 'managed' or not manifest['full_bag']:
        raise ValueError('Requires managed full-bag natural EOF')
    origin = manifest['start_stamp']
    duration = manifest['original_bag_end'] - origin
    trace, runner = rows(directory / 'frontend.jsonl'), rows(directory / 'runner_events.jsonl')
    pixels, gate = rows(directory / 'pixel_delivery.jsonl'), rows(directory / 'image_gate/gate.jsonl')
    services = sorted([row for path in sorted((directory / 'services').glob('*/service.jsonl')) for row in rows(path)], key=lambda row: row['wall'])
    eof = first(runner, ('playback_eof',))
    if eof is None:
        raise ValueError('Missing observed playback_eof; no inferred or fixed cutoff substitute')
    eof_wall, observation_end = eof['wall'], eof['wall'] + 3.
    shutdown = first(runner, ('experiment_shutdown',))
    if shutdown and shutdown['wall'] < observation_end:
        raise ValueError('Natural EOF observation ended before registered three wall seconds')
    analyzed = [row for row in trace if row['wall'] < observation_end]
    pre = [row for row in analyzed if row['wall'] < eof_wall]
    post = [row for row in analyzed if row['wall'] >= eof_wall]
    identity, consumed, ledger = base.identity_audit(analyzed, [])
    eof_identity, _, eof_ledger = base.identity_audit(pre, [])
    invariant, checks, postm = check_feedback(analyzed, manifest['feedback_mode'])
    calls = build_calls(directory.name, analyzed, pixels, consumed, postm, origin, eof_wall)
    precalls, postcalls = ([call for call in calls if call['wall'] < eof_wall],
                          [call for call in calls if call['wall'] >= eof_wall])
    policy_calls = [predicate_call(call, 'C_manager_success', origin) for call in calls]
    prepolicy = [call for call in policy_calls if call['wall'] < eof_wall]
    postpolicy = [call for call in policy_calls if eof_wall <= call['wall'] < observation_end]
    transitions, last = [], None
    for row in analyzed:
        if row['health_state'] != last:
            if row['clock'] >= origin:
                transitions.append(dict(point(row, origin), after_eof=row['wall'] >= eof_wall))
            last = row['health_state']
    phases = []
    for name, lo, hi in [('before_fault', 15, 43), ('fault_window', 43, 51),
                         ('fixed_recovery', 51, 61), ('late_to_EOF', 61, duration), ('attached_to_EOF', 15, duration)]:
        selected = [call for call in precalls if lo <= call['clock_relative_s'] < hi]
        selected_policy = [call for call in prepolicy if lo <= call['clock_relative_s'] < hi]
        selected_trace = [row for row in pre if origin + lo <= row['clock'] < origin + hi]
        value = dict(case=directory.name, phase=name, **counts(selected, lo, hi, selected_policy))
        value.update(lio=prior.intervals(selected_trace), withdrawals=sum(row['event'] == 'withdrawn' for row in selected_trace),
            transitions=[row for row in transitions if not row['after_eof'] and lo <= row['clock_relative_s'] < hi])
        phases.append(value)
    eof_state = dict(event=point(eof, origin), observation_end_wall=observation_end,
        last_pre_EOF=snapshot(pre[-1] if pre else None, origin), first_post_EOF=snapshot(post[0] if post else None, origin),
        last_post_EOF=snapshot(post[-1] if post else None, origin),
        last_lio_pre_EOF=point(next((row for row in reversed(pre) if row['event'] == 'lio'), None), origin),
        last_lio_post_EOF=point(next((row for row in reversed(post) if row['event'] == 'lio'), None), origin),
        pre_EOF_identity=eof_identity, outstanding_at_EOF=[row for row in eof_ledger if row['outcome'] in ('remaining', 'selected')],
        end_observation_remaining=identity['remaining_stamps'], post_EOF_real_calls=postcalls,
        calls_with_solver_and_policy_on_opposite_EOF_sides=[call for call in calls
            if call['after_eof'] != call['policy_after_eof']],
        post_EOF_policy_events=postpolicy,
        post_EOF_counts=counts(postcalls, policy_calls=postpolicy), post_EOF_lio=prior.intervals(post),
        pending_endpoints_note='Snapshot fields are the available evidence; exact pending IMU/LiDAR message identities are unknown unless independently recorded. No post-EOF call changes an EOF-before recovery verdict.')
    final_ledger = {row['corrected_stamp']: row for row in ledger}
    pending_outcomes = [dict(before=row, after=final_ledger.get(row['corrected_stamp']))
                        for row in eof_state['outstanding_at_EOF']]
    eof_state['outstanding_disposition'] = pending_outcomes
    eof_state['outstanding_disposition_counts'] = dict(Counter(
        row['after']['outcome'] if row['after'] else 'unknown' for row in pending_outcomes))
    eof_state['drain_status'] = ('not_applicable_no_outstanding_at_EOF' if not pending_outcomes else
        'all_outstanding_consumed' if all(row['after'] and row['after']['outcome'] == 'vio_processed'
            for row in pending_outcomes) else 'not_all_outstanding_consumed_clear_is_not_drain')
    eof_state['input_endpoints'] = input_endpoints(directory, pixels, analyzed, origin)
    fault = fault_recovery(directory.name, manifest, pre, precalls, runner, gate, pixels, services, origin, eof_wall)
    black = [call for call in precalls if call['content'] == 'black']
    mechanism = dict(black_identity_counts=counts(black), actual_black_pixels=sum(row['content'] == 'black' for row in pixels),
        black_calls_without_service=sum(not call['service_alive'] for call in black),
        black_calls_without_fresh_input=sum(not call['input_fresh'] for call in black),
        pixel_timestamps_strictly_monotonic=all(b['stamp_ns'] > a['stamp_ns'] for a, b in zip(pixels, pixels[1:])),
        service_started=[point(row, origin) for row in services if row['event'] == 'started'],
        service_stopped=[point(row, origin) for row in services if row['event'] == 'stopped'],
        service_sent_in_fault_window=sum(row['event'] == 'sent' and origin + 43 <= row['clock'] < origin + 51 for row in services),
        gate_last_counts=next((row for row in reversed(gate) if row['event'] in ('counts', 'stopped')), None),
        frozen_burst_schedule=manifest['recovery_schedule']=='burst')
    result = dict(case=directory.name, manifest=str(directory / 'manifest.json'), origin=origin,
        bag_duration_sensor_s=duration, natural_EOF=point(eof, origin), phases=phases,
        feedback_invariants=invariant, identity_audit=identity, fault_recovery=fault,
        withdrawals=withdrawals(pre, precalls, origin, eof_wall, manifest), transitions=transitions,
        EOF_observation=eof_state, mechanism=mechanism, whole_pre_EOF_lio=prior.intervals(pre),
        long_no_qualifying_call_intervals=long_gaps(pre, precalls, origin, duration, eof_wall),
        raw_calls=len(calls), before_EOF_calls=len(precalls), after_EOF_calls=len(postcalls),
        metric_contract=dict(A='Historical accepted/reverted finite-retained diagnostic, unchanged.',
            B='Finite surviving pose measurement and real covariance commitment; no pose-increment threshold.',
            C='Post-call actual policy_result_counted AND policy_result_success AND not ignored; attributed to actual visual_feedback wall/clock, while A/B use vio wall/clock.',
            sustained='Each predicate separately: >=3 consecutive matching VIO calls spanning >=0.2 corrected-image seconds; withdrawal/application/epoch stage resets.',
            boundary='Normal metrics use wall<first playback_eof, fixed sensor windows use bag-end minus bag-start. EOF+3wall is separate.',
            limits='Functional regression on the known sequence; finite information does not establish correspondence truth or physical recovery accuracy. PROBING changes the actual filter.'))
    write(out / (directory.name + '.json'), result)
    csv_write(out / (directory.name + '_calls.csv'), calls)
    csv_write(out / (directory.name + '_feedback_invariants.csv'), checks)
    csv_write(out / (directory.name + '_identity_ledger.csv'), ledger)
    write(out / (directory.name + '_raw_events.json'), dict(
        runner=runner, transitions=[row for row in analyzed if row['event'] in
            ('health_transition', 'applied', 'withdrawn', 'model_epoch_staged')],
        service_sessions=[row for row in services if row['event'] in ('started', 'stopped')]))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--cases', nargs='+', choices=CASES, default=list(CASES))
    args = parser.parse_args()
    root = args.root.resolve()
    out = (args.out or root / 'analysis').resolve()
    if out.exists():
        parser.error('Fresh output directory required; earlier analysis is never overwritten')
    out.mkdir(parents=True, exist_ok=False)
    results = [analyze_case(root / 'runs' / case, out) for case in args.cases]
    write(out / 'summary.json', dict(cases=args.cases, results=results,
        all_identity_checks_passed=all(row['identity_audit']['passed'] for row in results),
        all_feedback_invariants_passed=all(row['feedback_invariants']['passed'] for row in results),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), inputs_sha256=INPUTS,
        metric_modules_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in [Path(__file__).with_name(n) for n in ('metrics.py','queue_audit.py','summary_helpers.py')]}))
    csv_write(out / 'phases.csv', [phase for result in results for phase in result['phases']])
    overview = []
    for result in results:
        item = dict(case=result['case'], pre_EOF_calls=result['before_EOF_calls'], post_EOF_calls=result['after_EOF_calls'],
                    withdrawals=len(result['withdrawals']), identity_pass=result['identity_audit']['passed'],
                    feedback_pass=result['feedback_invariants']['passed'])
        for key, value in result['fault_recovery'].get('predicates', {}).items():
            item[key + '_recovery'] = value['status']
            item[key + '_first_clock_s'] = value['first']['clock_relative_s'] if value['first'] else None
            item[key + '_sustained_clock_s'] = value['achieved']['clock_relative_s'] if value['achieved'] else None
        overview.append(item)
    csv_write(out / 'overview.csv', overview)
    print(json.dumps(dict(output=str(out), overview=overview), indent=2))
    return 0 if all(row['identity_audit']['passed'] and row['feedback_invariants']['passed'] for row in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
