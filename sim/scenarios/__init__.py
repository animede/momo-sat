"""検証シナリオ S1〜S5（docs/04_implementation_plan.md §4、S4 は docs/08_thrust_extension.md §4、
S5 は docs/09_astro_observation_extension.md §4）。"""

from sim.scenarios.s1_detumble import build_scenario as build_s1
from sim.scenarios.s2_imaging import build_scenario as build_s2
from sim.scenarios.s3_power_thermal import build_scenario as build_s3
from sim.scenarios.s4_thrust import build_scenario as build_s4
from sim.scenarios.s5_observe import build_scenario as build_s5

SCENARIOS = {
    "S1": build_s1,
    "S2": build_s2,
    "S3": build_s3,
    "S4": build_s4,
    "S5": build_s5,
}
