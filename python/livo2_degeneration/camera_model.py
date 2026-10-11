"""Validation for a given full-resolution camera model and CameraInfo publication."""
import numpy as np


def validate_model(model):
    if model['width'] <= 0 or model['height'] <= 0 or not model['frame_id']:
        raise ValueError('Invalid dataset image dimensions or optical frame')
    if model['distortion_model'] != 'plumb_bob':
        raise ValueError('Unsupported dataset camera model')
    for key, size in [('K', 9), ('R', 9), ('P', 12)]:
        if len(model[key]) != size or not np.isfinite(model[key]).all():
            raise ValueError('Invalid camera matrix ' + key)
    if len(model['D']) not in (4, 5) or not np.isfinite(model['D']).all():
        raise ValueError('Invalid distortion coefficients')
    K = np.asarray(model['K']).reshape(3, 3)
    P = np.asarray(model['P']).reshape(3, 4)
    if K[0, 0] <= 0 or K[1, 1] <= 0 or not np.allclose(K[2], [0, 0, 1]):
        raise ValueError('Invalid full-resolution intrinsic matrix')
    if not np.allclose(P[:, :3], K) or not np.allclose(P[:, 3], 0):
        raise ValueError('Projection matrix must match the full-resolution color K')
    if not np.allclose(np.asarray(model['R']).reshape(3, 3), np.eye(3)):
        raise ValueError('Aligned color model must not introduce a second rectification')


def validate_image(model, width, height, frame_id, stamp_ns, encoding):
    if (width, height) != (model['width'], model['height']):
        raise ValueError('Image dimensions disagree with the fixed dataset intrinsics')
    if frame_id != model['frame_id']:
        raise ValueError('Image optical frame disagrees with the dataset model')
    if stamp_ns <= 0 or encoding not in ('rgb8', 'bgr8'):
        raise ValueError('Invalid color timestamp or encoding')
