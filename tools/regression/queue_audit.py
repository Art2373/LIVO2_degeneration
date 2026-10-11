"""Exact decoded/enqueued/selected/consumed identity ledger."""
import math
from collections import Counter
from metrics import cache_state,actual_vio
EPS=1e-6

def identity_audit(trace, episodes):
    """Exact source ledger, adapted from frontend_recovery/audit_consumed_images.

    Only multi-episode content labels differ. Queue identity uses the serialized
    doubles exactly, without timestamp rounding or a nearest-frame substitute.
    """
    ledger, queue, consumed, clears, violations = {}, [], [], [], []
    selected = dispatched = None
    last_clear_line, last_clear_wall = 0, -math.inf
    max_queue = 0

    def issue(row, reason, **data):
        violations.append(dict(line=row['_line'], event=row['event'], reason=reason, **data))

    def remove(row, outcome, before=False):
        stamp = row['event_corrected']
        if stamp not in ledger or stamp not in queue:
            issue(row, 'removed_identity_not_in_queue', stamp=stamp)
            return None
        if before and row['image_queue'] != len(queue):
            issue(row, 'pre_removal_queue_size_mismatch', reconstructed=len(queue))
        queue.remove(stamp)
        ledger[stamp].update(outcome=outcome, outcome_line=row['_line'], outcome_wall=row['wall'])
        return ledger[stamp]

    for row in trace:
        event = row['event']
        if event == 'image_decoded':
            stamp = row['event_corrected']
            if stamp in ledger:
                issue(row, 'duplicate_decoded_identity', stamp=stamp)
            ledger[stamp] = dict(corrected_stamp=stamp, raw_stamp=row['event_raw'], decoded_line=row['_line'],
                                queued_line=None, queued_wall=None, model_epoch=None, outcome='remaining',
                                outcome_line=None, outcome_wall=None, selected_line=None, dispatched_line=None, vio_line=None)
            queue.append(stamp)
        elif event == 'image_queued':
            stamp = row['event_corrected']
            if stamp not in ledger:
                issue(row, 'queued_without_decode', stamp=stamp)
            else:
                ledger[stamp].update(queued_line=row['_line'], queued_wall=row['wall'], model_epoch=row.get('model_epoch'))
        elif event in ('image_drop_overflow', 'image_drop_sync_past'):
            remove(row, 'overflow' if event == 'image_drop_overflow' else 'stale', True)
        elif event in ('image_selected', 'image_skip_lidar_points', 'image_skip_imu_init'):
            item = remove(row, 'selected' if event == 'image_selected' else event)
            if item is not None and event == 'image_selected':
                if selected is not None:
                    issue(row, 'second_pending_selection')
                item['selected_line'] = row['_line']
                selected = item['corrected_stamp']
        elif event == 'vio_dispatched':
            stamp = row['event_corrected']
            if selected != stamp:
                issue(row, 'dispatch_does_not_match_selected', expected=selected, actual=stamp)
            if stamp in ledger:
                ledger[stamp]['dispatched_line'] = row['_line']
            selected, dispatched = None, stamp
        elif event in ('applied', 'withdrawn'):
            last_clear_line, last_clear_wall = row['_line'], row['wall']
            clears.append(dict(line=row['_line'], wall=row['wall'], event=event, discarded_stamps=list(queue),
                               snapshot=cache_state(row)))
            for stamp in queue:
                ledger[stamp].update(outcome='cleared_' + event, outcome_line=row['_line'], outcome_wall=row['wall'])
            queue.clear()
            if selected is not None:
                ledger[selected].update(outcome='selected_then_cleared_' + event, outcome_line=row['_line'], outcome_wall=row['wall'])
            selected = dispatched = None
        elif event == 'vio':
            stamp = row.get('vio_image_corrected_time')
            item = ledger.get(stamp)
            if item is None:
                issue(row, 'vio_identity_never_decoded', stamp=stamp)
                continue
            checks = dict(prior_queued_identity=item['queued_line'] is not None and item['queued_line'] < row['_line'],
                          selected_and_dispatched=item['selected_line'] is not None and item['dispatched_line'] is not None and dispatched == stamp,
                          not_dropped_or_cleared=item['outcome'] == 'selected', first_consumption=item['vio_line'] is None,
                          after_last_clear=item['queued_line'] is not None and item['queued_line'] > last_clear_line and item['queued_wall'] >= last_clear_wall,
                          same_model_epoch=item['model_epoch'] == row.get('model_epoch'), state_timestamp_exact=row['state_time'] == stamp)
            for key, good in checks.items():
                if not good:
                    issue(row, key, stamp=stamp)
            item.update(vio_line=row['_line'], outcome='vio_processed')
            consumed.append(dict(line=row['_line'], wall=row['wall'], sensor_stamp=stamp, raw_stamp=item['raw_stamp'],
                                 queued_line=item['queued_line'], queued_wall=item['queued_wall'], last_clear_line=last_clear_line,
                                 model_epoch=row.get('model_epoch'), retained_finite=actual_vio(row), **checks))
            dispatched = None
        elif event == 'vio_empty_lidar':
            if dispatched in ledger:
                ledger[dispatched]['outcome'] = 'dispatched_but_empty_lidar'
            dispatched = None
        if event != 'image_drop_overflow':
            max_queue = max(max_queue, row.get('image_queue', 0))
            if row.get('image_queue', 0) > 8:
                issue(row, 'stable_queue_exceeds_eight')
        if event in ('image_queued', 'image_selected', 'image_skip_lidar_points', 'image_skip_imu_init', 'applied', 'withdrawn'):
            if len(queue) != row['image_queue']:
                issue(row, 'stable_reconstructed_queue_size', reconstructed=len(queue))
            if queue and (queue[0] != row['queue_first_stamp'] or queue[-1] != row['queue_last_stamp']):
                issue(row, 'stable_reconstructed_queue_endpoints')
    for row in consumed:
        row['drop_interval_ids'] = []
        row['legal_pre_fault_tail_ids'] = []
        for episode in episodes:
            if episode['kind'] != 'rgb' or not episode['actual_begin']:
                continue
            begin = episode['actual_begin']['message_stamp']
            end = episode['actual_end']['message_stamp'] if episode['actual_end'] else math.inf
            if begin - EPS <= row['raw_stamp'] < end - EPS:
                row['drop_interval_ids'].append(episode['id'])
                violations.append(dict(line=row['line'], reason='consumed_image_inside_actual_drop_interval', episode=episode['id'], raw_stamp=row['raw_stamp']))
            if (episode['begin']['wall'] <= row['wall'] < (episode['end']['wall'] if episode['end'] else math.inf)
                    and row['raw_stamp'] < begin - EPS):
                row['legal_pre_fault_tail_ids'].append(episode['id'])
    stamps = [row['sensor_stamp'] for row in consumed]
    unique = len(set(stamps)) == len(stamps)
    monotonic = all(b > a for a, b in zip(stamps, stamps[1:]))
    if not unique or not monotonic:
        violations.append(dict(reason='consumed_images_not_unique_strictly_monotonic'))
    summary = dict(passed=not violations, violations=violations, admitted=len(ledger),
                   ledger_outcomes=dict(Counter(row['outcome'] for row in ledger.values())),
                   vio_consumptions=len(consumed), unique=unique, strictly_monotonic=monotonic,
                   maximum_stable_queue=max_queue, remaining_stamps=queue,
                   clears=clears, identity_definition='Exact parsed 17-digit frontend doubles; 1us tolerance only for cross-process gate boundary labels')
    return summary, consumed, list(ledger.values())
