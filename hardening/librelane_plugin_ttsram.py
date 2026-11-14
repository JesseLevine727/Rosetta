"""
LibreLane plugin: run the tile's Metal4 power stripes across hard macros
(IHP SG13CMOS5L SRAMs) after the PDN is generated.

LibreLane imports every module on the Python path whose name starts with
``librelane_plugin_``.  ``python -m librelane`` has the current directory on
the path, and tt_tool.py --harden (locally and in the Tiny Tapeout GDS action)
runs it from the repository root, inside the LibreLane container, so this file
is picked up without installing anything.  src/config.json inserts the step
right after OpenROAD.GeneratePDN:

    "meta": {"substituting_steps": {"+OpenROAD.GeneratePDN": "Project.ExtendStripesOverMacros"}}

The work is done by odb_extend_stripes.py (see its docstring).  The approach
follows kdp1965/ihp-um-janestreet-prism (Project.ExtendPowerStripes), written
from scratch in a much smaller form.
"""
import os
from decimal import Decimal
from typing import List

from librelane.config import Variable
from librelane.steps import Step
from librelane.steps.odb import OdbpyStep

HERE = os.path.dirname(os.path.abspath(__file__))


@Step.factory.register()
class ExtendStripesOverMacros(OdbpyStep):
    id = "Project.ExtendStripesOverMacros"
    name = "Extend Power Stripes Over Macros"

    config_vars = [
        Variable("EXTEND_STRIPES_LAYER", str, "Layer of the tile's vertical stripes.", default="Metal4"),
        Variable("EXTEND_STRIPES_POWER_NET", str, "Power net of the stripes.", default="VPWR"),
        Variable("EXTEND_STRIPES_GROUND_NET", str, "Ground net of the stripes.", default="VGND"),
        Variable(
            "EXTEND_STRIPES_CLEARANCE",
            Decimal,
            "Spacing kept between a stripe and the other supply net's Metal4 pins of a macro.",
            units="µm",
            default=Decimal("0.3"),
        ),
        Variable(
            "EXTEND_STRIPES_STRIPE_CLEARANCE",
            Decimal,
            "Spacing kept between two stripes when one is moved.",
            units="µm",
            default=Decimal("1.0"),
        ),
        Variable(
            "EXTEND_STRIPES_MAX_NUDGE",
            Decimal,
            "Largest sideways move of a stripe to make it legal or put it on a supply column of its net.",
            units="µm",
            default=Decimal("15"),
        ),
        Variable(
            "EXTEND_STRIPES_PREFER_CONNECT",
            bool,
            "Move a legal stripe that touches none of the macro's columns of its own net onto one (within the max nudge).",
            default=True,
        ),
    ]

    def get_script_path(self):
        return os.path.join(HERE, "odb_extend_stripes.py")

    def get_command(self) -> List[str]:
        cmd = super().get_command() + [
            "--layer",
            self.config["EXTEND_STRIPES_LAYER"],
            "--power-net",
            self.config["EXTEND_STRIPES_POWER_NET"],
            "--ground-net",
            self.config["EXTEND_STRIPES_GROUND_NET"],
            "--clearance",
            str(self.config["EXTEND_STRIPES_CLEARANCE"]),
            "--stripe-clearance",
            str(self.config["EXTEND_STRIPES_STRIPE_CLEARANCE"]),
            "--max-nudge",
            str(self.config["EXTEND_STRIPES_MAX_NUDGE"]),
            "--report",
            os.path.join(self.step_dir, "stripes.json"),
            "--check-grid-dir",
            self.step_dir,
        ]
        if not self.config["EXTEND_STRIPES_PREFER_CONNECT"]:
            cmd.append("--no-prefer-connect")
        return cmd
