from csaf_f16.systems import F16Simple
from csaf_f16.messages import *
from csaf_f16.components import F16PlantComponent

from f16dynamics.f16dynpy import F16Plant
import numpy as np

# create context for the optimized model

f16 = F16Plant()

def model_state_update(model, time_t, state_f16, input_controller):
    """the new optimized state evolution function"""
    f16 = F16Plant()
    s = np.array(state_f16)
    u = np.array(input_controller)
    return f16.dxdt(s, u)


def model_output(model, time_t, state_f16, input_controller):
    """the new optimized output function"""
    s = np.array(state_f16)
    u = np.array(input_controller)
    return f16.output(s, u)


class F16FastPlantComponent(F16PlantComponent):
    """wrap the optimized methods into a CSAF component"""
    flows = {
        "outputs": model_output,
        "states": model_state_update
    }


class F16FastSimple(F16Simple):
    """place the component into a simple controls architecture"""
    components = {
        **F16Simple.components,
        "plant": F16FastPlantComponent,

    }