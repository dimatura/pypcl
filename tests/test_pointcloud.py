import pytest

import pypcl

def test_pointcloudxyz():
    pc = pypcl.PointCloudXYZ()


def test_pointcloudxyz():
    pc = pypcl.PointCloudXYZRGB()


def test_load():
    pc = pypcl.load_pcd_pcxyz('test_data/bincomp.pcd')
    assert(pc.size() == 3157)
    assert(pc.width == 3157)
    assert(pc.height == 1)
