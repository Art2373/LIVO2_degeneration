"""Read-only common metric definitions."""
import math
from collections import Counter
import numpy as np
import metrics as old

def point(r,origin):
 if r is None:return None
 keys=['_line','event','wall','clock','state_time','feedback_call_id','vio_image_raw_time','vio_image_corrected_time','raw_stamp','stamp_ns','health_state','policy_reason','policy_event_reason','service_alive','model_usable','input_fresh','consecutive_failures','consecutive_successes','policy_last_success_time','content','command_id','message_stamp','feedback_eligible','policy_result_counted','policy_result_success']
 d={k:r[k] for k in keys if k in r};d['clock_relative_s']=r['clock']-origin if r.get('clock',0)>0 else None
 return d

def intervals(rows):
 lio=[r for r in rows if r['event']=='lio'];stamps=[r['state_time'] for r in lio]
 ds=np.diff(stamps);dw=np.diff([r['wall'] for r in lio]);dc=np.diff([r['clock'] for r in lio])
 return dict(calls=len(lio),effective=sum(r.get('lio_steps',0)>0 for r in lio),duplicates=int(sum(ds==0)),backwards=int(sum(ds<0)),
  nonfinite=sum(not bool(r.get('state_finite',False)) or not all(math.isfinite(float(v)) for v in r.get('position',[])+r.get('quaternion_xyzw',[])) for r in lio),
  state_intervals_s=old.distribution(ds),wall_intervals_s=old.distribution(dw),clock_intervals_s=old.distribution(dc))

def counts(calls,lo=None,hi=None):
 def bins(key):return sorted({math.floor(r['clock_relative_s']) for r in calls if r[key]})
 legacy=sum(r['legacy_retained'] for r in calls);eligible=sum(r['eligible'] for r in calls)
 result=dict(vio_calls=len(calls),legacy_retained=legacy,legacy_not_retained=len(calls)-legacy,eligible=eligible,
  policy_counted=sum(r['policy_counted'] for r in calls),policy_success=sum(r['policy_success'] for r in calls),policy_non_success=sum(r['policy_counted'] and not r['policy_success'] for r in calls),
  policy_ignored=sum(r['policy_ignored'] for r in calls),zero_observation=sum(r['visual_observations']==0 for r in calls),
  zero_pose_information_retained=sum(r['zero_pose_info_retained'] for r in calls),zero_pose_info_success_credit=sum(r['zero_pose_info_retained'] and r['policy_success'] for r in calls),
  zero_pose_info_refresh=sum(r['zero_pose_info_retained'] and r['success_time_refreshed'] for r in calls),zero_pose_info_failure_cleared=sum(r['zero_pose_info_retained'] and r['failures_cleared'] for r in calls),
  zero_increment_eligible=sum(r['zero_pose_increment'] and r['eligible'] for r in calls),zero_increment_success_credit=sum(r['zero_pose_increment'] and r['policy_success'] for r in calls),
  service_not_alive=sum(not r['service_alive'] for r in calls),input_not_fresh=sum(not r['input_fresh'] for r in calls),model_not_usable=sum(not r['model_usable'] for r in calls),
  content_counts=dict(Counter(r['content'] for r in calls)),eligibility_reasons=dict(Counter(r['eligibility_reason'] for r in calls)),policy_reasons=dict(Counter(r['policy_event_reason'] for r in calls)))
 if lo is not None:
  result.update(denominator_sensor_seconds=hi-lo,bin_definition='one sensor-/clock-second bin [lo,hi); fixed interval, not number of scheduled calls',legacy_retained_bins=bins('legacy_retained'),eligible_bins=bins('eligible'),policy_success_bins=bins('policy_success'))
  result.update(legacy_retained_coverage=len(result['legacy_retained_bins'])/(hi-lo),eligible_coverage=len(result['eligible_bins'])/(hi-lo),policy_success_coverage=len(result['policy_success_bins'])/(hi-lo))
 return result
