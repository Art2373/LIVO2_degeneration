#!/usr/bin/env python3
"""Serial four-case worker; never overwrites or automatically retries a run."""
import argparse,json,subprocess,sys,time
from pathlib import Path

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--plan',type=Path,required=True);p.add_argument('--execute',action='store_true');a=p.parse_args()
 plan=json.loads(a.plan.read_text());root=Path(plan['root']);path=root/'batch_summary.json'
 if path.exists():p.error('Batch evidence exists; no retry/overwrite')
 for case in plan['run_order']:
  if (root/'runs'/case).exists():p.error('Run output exists; no retry/overwrite: '+case)
 if not a.execute:print(json.dumps(dict(plan=str(a.plan.resolve()),order=plan['run_order'],execution=False)));return 0
 summary=dict(plan=str(a.plan.resolve()),started_wall=time.time(),status='running',runs=[])
 def save():
  temp=path.with_suffix('.tmp');temp.write_text(json.dumps(summary,indent=2));temp.replace(path)
 save()
 for case in plan['run_order']:
  command=[sys.executable,str(Path(__file__).with_name('run.py')),'--plan',str(a.plan.resolve()),'--case',case]
  with (root/('worker_'+case+'.log')).open('x') as stream:
   before=time.time();process=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT)
  manifest=root/'runs'/case/'manifest.json';record=json.loads(manifest.read_text()) if manifest.exists() else {}
  entry=dict(case=case,command=command,returncode=process.returncode,wall_start=before,wall_end=time.time(),manifest_complete=record.get('completed',False));summary['runs'].append(entry);save()
  if process.returncode!=0 or not entry['manifest_complete']:
   summary.update(status='stopped_on_failure',finished_wall=time.time());save();return 1
 summary.update(status='complete',finished_wall=time.time());save();return 0
if __name__=='__main__':raise SystemExit(main())
