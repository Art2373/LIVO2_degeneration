#!/usr/bin/env python3
"""Targeted model/black-injection/clock-definition checks without a ROS master."""
import copy,json,math,sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'python'))
from livo2_degeneration.fusion_model import load_reference
from livo2_degeneration.camera_model import validate_model,validate_image
from image_gate import black_message
from inputs import header
from metrics import actual_vio
class Runtime(unittest.TestCase):
 def test_supplied_model_transform_and_single_time_offset(self):
  model=load_reference(ROOT/'config/m3dgr/reference.yaml',ROOT/'config/m3dgr/camera_info.yaml',ROOT/'config/m3dgr/common.yaml')
  self.assertLess(np.max(np.abs(np.array(model['T_IC_applied'])@np.array(model['T_CL'])-np.array(model['T_IL']))),2e-6)
  self.assertEqual(model['offset'],.1)
  self.assertEqual(model['camera']['width'],640)
  self.assertAlmostEqual(model['camera']['K'][0],607.79772949218)
 def test_camera_contract_rejects_scale_and_wrong_frame(self):
  import yaml
  model=yaml.safe_load((ROOT/'config/m3dgr/camera_info.yaml').read_text());validate_model(model)
  validate_image(model,640,480,model['frame_id'],100,'rgb8')
  with self.assertRaises(ValueError):validate_image(model,320,240,model['frame_id'],100,'rgb8')
  with self.assertRaises(ValueError):validate_image(model,640,480,'body',100,'rgb8')
 def test_black_changes_only_payload_not_original_or_identity(self):
  msg=SimpleNamespace(encoding='rgb8',data=bytes(range(12)),header=SimpleNamespace(stamp=123,frame_id='optical'),width=2,height=2,step=6,is_bigendian=0)
  frozen=copy.deepcopy(msg);black=black_message(msg)
  self.assertEqual(vars(msg),vars(frozen));self.assertEqual(black.data,bytes(12));self.assertIs(black.header,msg.header)
  self.assertEqual({k:v for k,v in vars(black).items() if k!='data'},{k:v for k,v in vars(msg).items() if k!='data'})
 def test_historical_retention_does_not_imply_information(self):
  r=dict(event='vio',vio_steps=1,vio_reverted=0,vio_pixels=100,vio_retained_finite=True,feedback_eligible=False)
  self.assertTrue(actual_vio(r));self.assertFalse(r['feedback_eligible'])
 def test_raw_header_nanoseconds_not_float_identity(self):
  import struct
  data=struct.pack('<IIII',1,1732437676,878834234,4)+b'body'
  self.assertEqual(header(data),(1,1732437676878834234,'body',20))
if __name__=='__main__':unittest.main(verbosity=2)
