#!/usr/bin/env python3
"""Read-only dependency/image-conversion smoke check in tools/env.sh."""
import argparse,json,os,re,subprocess
from pathlib import Path
import cv2
import numpy as np
from cv_bridge import CvBridge
import cv_bridge
from livo2_degeneration.msg import FusionModel
from livox_ros_driver2.msg import CustomMsg

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,default=Path(os.environ['LIVO2_WS']))
    a=parser.parse_args();ws=a.workspace.resolve()
    bridge=CvBridge();rgb=np.arange(72,dtype=np.uint8).reshape(4,6,3)
    message=bridge.cv2_to_imgmsg(rgb,encoding='rgb8')
    recovered=bridge.imgmsg_to_cv2(message,desired_encoding='rgb8')
    bgr=bridge.imgmsg_to_cv2(message,desired_encoding='bgr8')
    assert np.array_equal(recovered,rgb) and np.array_equal(bgr,rgb[:,:,::-1])
    gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
    resized=cv2.resize(gray,(3,2));grad=cv2.Sobel(gray,cv2.CV_32F,1,0)
    assert resized.shape==(2,3) and np.isfinite(grad).all()
    executable=ws/'devel/lib/livo2_degeneration/fusion_mapping'
    linked=subprocess.check_output(['ldd',str(executable)],text=True)
    assert 'not found' not in linked
    core={}
    for line in linked.splitlines():
        m=re.search(r'(libdegeneration_[^ ]+) => ([^ ]+)',line)
        if m:
            resolved=Path(m.group(2)).resolve()
            assert ws in resolved.parents, str(resolved)
            core[m.group(1)]=str(resolved)
    assert len(core)==5,core
    assert ws in Path(cv_bridge.__file__).resolve().parents
    records=dict(workspace=str(ws),opencv_version=cv2.__version__,cv2_module=cv2.__file__,
        cv_bridge_module=cv_bridge.__file__,fusion_model_type=FusionModel._type,
        livox_type=CustomMsg._type,livox_md5=CustomMsg._md5sum,
        core_libraries=core,rgb_conversion=True,gray_resize_sobel=True,
        loaded_opencv_libraries=sorted(set(line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines() if 'libopencv_' in line)))
    assert FusionModel._type=='livo2_degeneration/FusionModel'
    assert CustomMsg._type=='livox_ros_driver2/CustomMsg'
    print(json.dumps(records,indent=2))
if __name__=='__main__':main()
