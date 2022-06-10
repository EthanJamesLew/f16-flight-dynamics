import csaf.utils as csafutils
import numpy as np


def test_f16simple_csaf():
    """test that the fast version of CSAF F16 Simple matches within tolerance"""
    from csaf_f16.systems import F16Simple
    from f16dynamics.csaf import F16FastSimple

    # error that we're willing to accept over 20s
    abs_tol_20s = 0.4

    sys = F16Simple()
    sys_fast = F16FastSimple()
    trajs = sys.simulate_tspan((0.0, 20.0))
    trajs_fast = sys_fast.simulate_tspan((0.0, 20.0))
    assert np.all(np.isclose(np.array(trajs["plant"].states), np.array(trajs_fast["plant"].states), atol=abs_tol_20s))
