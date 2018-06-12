import numpy as np

import libpypcl


__all__ = ['pclpc2_from_ndarray',
           'xyz_from_structured_array',
           'rgb_from_structured_array',
           ]


def pclpc2_from_ndarray(arr):
    # TODO for some reason can't do this entirely in c++
    fields = [(n,)+arr.dtype.fields[n] for n in arr.dtype.names]
    return libpypcl._pclpc2_from_ndarray(arr, fields)


def xyz_from_structured_array(arr):
    for fld in 'xyz':
        if not fld in arr.dtype.fields:
            raise ValueError('input lacks %s field' % fld)
    out = np.empty((len(arr), 3), dtype='f4')
    out[:, 0] = arr['x']
    out[:, 1] = arr['y']
    out[:, 2] = arr['z']
    return out


def rgb_from_structured_array(arr, decode_rgb=True):

    def has_encoded_rgb(_arr):
        return 'rgb' in _arr.dtype.fields

    def has_separate_rgb(_arr):
        for fld in 'rgb':
            if fld not in arr.dtype:
                return False
        return True

    if not (has_encoded_rgb(arr) or has_separate_rgb(arr)):
        return ValueError('input lacks rgb')

    if decode_rgb:
        if has_encoded_rgb(arr):
            rgbtmp = arr['rgb'].copy().view('u1')
            rgbdec = rgbtmp.reshape((-1, 4))[:, :3][:, ::-1]
            return rgbdec.copy()
        else:
            out = np.empty((len(arr), 3), dtype='u1')
            out[:, 0] = arr['r']
            out[:, 1] = arr['g']
            out[:, 2] = arr['b']
            return out
    else:
        if has_encoded_rgb(arr):
            return arr['rgb'].copy()
        else:
            r = arr['r'].astype('u4')
            g = arr['g'].astype('u4')
            b = arr['b'].astype('u4')
            rgb = np.array((r << 16) | (g << 8) | d,
                           dtype='u4')
            return rgb
