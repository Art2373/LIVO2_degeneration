#!/usr/bin/env python3
"""Freeze the existing four-case full Wheel migration regression without replay."""
import argparse,datetime,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
import yaml

def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for data in iter(lambda:f.read(1024*1024),b''):h.update(data)
 return h.hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__)
 for name in ('bag','workspace','output','env'):p.add_argument('--'+name,required=True,type=Path)
 p.add_argument('--contract',type=Path);p.add_argument('--port',type=int,default=11331)
 a=p.parse_args()
 if os.environ.get('LIVO2_REGRESSION_ENV')!=str(a.env.resolve()):
  e=dict(os.environ,LIVO2_REGRESSION_ENV=str(a.env.resolve()))
  return subprocess.call([str(a.env.resolve()),sys.executable,str(Path(__file__).resolve())]+sys.argv[1:],env=e)
 from inputs import inspect_bag,image_contract
 repo=Path(__file__).resolve().parents[2];ws=a.workspace.resolve();bag=a.bag.resolve();root=a.output.resolve()
 if not bag.is_file() or root.exists():p.error('Readable bag and fresh output directory required')
 if not 1024<=a.port<=65535:p.error('Dedicated master port out of range')
 preflight=inspect_bag(bag)
 binary=ws/'devel/lib/livo2_degeneration/fusion_mapping'
 libs=[ws/('devel/lib/libdegeneration_'+n+'.so') for n in ('laser_mapping','vio','lio','imu_proc','pre')]
 for f in [binary]+libs:
  if not f.is_file():raise RuntimeError('Fresh independent build output missing: '+str(f))
 # This registered plan applies only to the already admitted Wheel-float01 input.
 bagsha=sha(bag)
 if bagsha!='366171a5f1d0c1e7bd602e2b664d85dbd89dd08570c4165142ef9849029c1cf6':raise RuntimeError('Wheel-float01 source checksum differs; this fixed regression does not admit another sequence')
 root.mkdir(parents=True,exist_ok=False)
 contract=root/'input_contract.json'
 if a.contract:
  copied=json.loads(a.contract.read_text());assert copied.get('image_hashes')
  contract.write_text(json.dumps(copied,indent=2))
 else:contract.write_text(json.dumps(image_contract(bag),indent=2))
 frozen={}
 for folder in ('src','include','msg','python','scripts','launch','config','tools'):
  for f in sorted((repo/folder).rglob('*')):
   if f.is_file() and '__pycache__' not in f.parts:frozen[str(f)]=sha(f)
 for name in ('CMakeLists.txt','package.xml','setup.py'):
  f=repo/name
  if f.exists():frozen[str(f)]=sha(f)
 cfg=yaml.safe_load((repo/'config/management.yaml').read_text())['dynamic_visual']
 policy={k:v for k,v in cfg.items() if k not in ('management_enabled','feedback_mode','recovery_schedule','schedule_trace_enabled')}
 cases={}
 for name,kind in [('H',None),('R','rgb'),('B','occlusion'),('S','service')]:
  cases[name]=dict(policy='managed',feedback='measurement_commit_v1',schedule='burst',events=[] if kind is None else [dict(episode_id=kind+'1',kind=kind,start_after_s=43,restore_after_s=51)])
 freeze=dict(current_sources=frozen,binaries={str(f):sha(f) for f in [binary]+libs},configuration={})
 (root/'RUN_FREEZE.json').write_text(json.dumps(freeze,indent=2))
 metadata=dict(preflight['metadata'],path=str(bag),size=bag.stat().st_size,sha256=bagsha)
 plan=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),root=str(root),repository=str(repo),workspace=str(ws),environment=str(a.env.resolve()),master_port=a.port,bag=str(bag),bag_metadata=metadata,observer_contract=str(contract),freeze=str(root/'RUN_FREEZE.json'),diagnostic_binary=str(binary),diagnostic_build_manifest=str(root/'RUN_FREEZE.json'),full_bag=True,rate=.5,attach_after_s=15,eof_observe_wall_s=3,policy_parameters=policy,cases=cases,run_order=['H','R','B','S'],max_replays=4,stop='Execution/injection error stops subsequent cases; no automatic retries. Strategy outcome does not change the fixed schedule.',metrics=dict(predicates=['A historical retained','B pose measurement information and finite commit','C actual manager credit'],sustained='At least 3 consecutive qualifying calls spanning at least 0.2 corrected image seconds; application/withdrawal resets.',windows=[[15,43],[43,51],[51,61],[61,metadata['duration']]],eof='Natural EOF; 3 wall seconds separately recorded.'),contract_source=str(a.contract.resolve()) if a.contract else 'Generated from compressed input')
 (root/'RUN_PLAN.json').write_text(json.dumps(plan,indent=2));(root/'input_preflight.json').write_text(json.dumps(preflight,indent=2))
 print(root/'RUN_PLAN.json');return 0
if __name__=='__main__':raise SystemExit(main())
