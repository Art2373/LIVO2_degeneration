"""Read and validate dataset-given fusion parameters; no online estimator."""
import hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

def load_reference(reference, camera, common):
    r = yaml.safe_load(Path(reference).read_text())
    c = yaml.safe_load(Path(camera).read_text())
    base = yaml.safe_load(Path(common).read_text())
    from livo2_degeneration.camera_model import validate_model
    validate_model(c)
    tcl = np.eye(4)
    tcl[:3, :3] = np.array(r['extrin_calib']['Rcl']).reshape(3, 3)
    tcl[:3, 3] = r['extrin_calib']['Pcl']
    til = np.eye(4)
    til[:3, :3] = np.array(base['extrin_calib']['extrinsic_R']).reshape(3, 3)
    til[:3, 3] = base['extrin_calib']['extrinsic_T']
    tic = til @ np.linalg.inv(tcl)
    from_matrix = getattr(Rotation, 'from_matrix', None) or Rotation.from_dcm
    quat = from_matrix(tic[:3, :3]).as_quat()
    applied = tic.copy()
    rotation = Rotation.from_quat(quat)
    as_matrix = getattr(rotation, 'as_matrix', None) or rotation.as_dcm
    applied[:3, :3] = as_matrix()
    effective_tcl = np.linalg.inv(applied) @ til
    offset = float(r['time_offset']['img_time_offset'])
    if (not np.isfinite(tic).all() or not np.isfinite(offset) or
            np.linalg.norm(tic[:3, 3]) > base['fusion_model']['max_baseline']):
        raise ValueError('Invalid given transform or time offset')
    # A failure here is a reference contract contradiction, not permission to fit it.
    if np.max(np.abs(effective_tcl-tcl)) > 2e-6:
        raise ValueError('Reference rotation is not sufficiently orthonormal')
    for key, idx in [('cam_fx', 0), ('cam_fy', 4), ('cam_cx', 2), ('cam_cy', 5)]:
        if float(r['laserMapping'][key]) != float(c['K'][idx]):
            raise ValueError('Reference and CameraInfo intrinsics disagree')
    return dict(source='dataset_given_reference', T_CL=tcl.tolist(), T_IL=til.tolist(),
                T_IC_matrix=tic.tolist(), T_IC_applied=applied.tolist(), quaternion_xyzw=quat.tolist(),
                effective_T_CL=effective_tcl.tolist(), rotation_rounding_max=float(np.max(np.abs(effective_tcl-tcl))),
                camera=c, offset=offset, revision=1,
                files={str(p): hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in (reference, camera, common)},
                reference_sha256=hashlib.sha256(Path(reference).read_bytes()).hexdigest())
