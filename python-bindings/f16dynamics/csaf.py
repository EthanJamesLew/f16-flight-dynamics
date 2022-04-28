from csaf_f16.systems import F16Simple
from csaf_f16.messages import *
from csaf_f16.components import F16PlantComponent, F16LlcComponent

from f16dynamics.f16dynpy import F16Plant, LowLevelController
import numpy as np

# create context for the optimized model
f16 = F16Plant()
llc = LowLevelController()


def create_model_update(comp):
    def _model_state_update(model, time_t, cstate, cinput):
        s = np.array(cstate)
        u = np.array(cinput)
        return comp.dxdt(s, u)
    return _model_state_update


def create_model_output(comp):
    def _model_output(model, time_t, cstate, cinput):
        s = np.array(cstate)
        u = np.array(cinput)
        return comp.output(s, u)
    return _model_output


def create_fast_component(comp, csaf_comp_type):
    """create csaf component from fast methods"""
    class _FastComponent(csaf_comp_type):
       """wrap the optimized methods into a CSAF component"""
       name = f"Fast {csaf_comp_type.name}"

       flows = {
           "outputs": create_model_output(comp),
           "states": create_model_update(comp)
       }
    return _FastComponent


F16FastPlantComponent = create_fast_component(f16, F16PlantComponent)


F16FastLlcComponent = create_fast_component(llc, F16LlcComponent)


class F16FastSimple(F16Simple):
    """place the component into a simple controls architecture"""
    components = {
        **F16Simple.components,
        "plant": F16FastPlantComponent,
        "llc": F16FastLlcComponent
    }