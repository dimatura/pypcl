
import libpypcl
from libpypcl import *

def pclpc2_from_ndarray(arr):
    # TODO for some reason can't do this entirely in c++
    fields = [(n,)+arr.dtype.fields[n] for n in arr.dtype.names]
    return libpypcl._pclpc2_from_ndarray(arr, fields)
