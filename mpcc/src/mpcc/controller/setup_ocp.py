from typing import Literal
from acados_template import AcadosOcp, ACADOS_INFTY
import numpy as np

def add_constraints(ocp: AcadosOcp, constraints_config: dict[str, any], controller_type: Literal["pt","mpcc", "empcc"]):
    
    # cross-wise constraints (will want to be set as a "safety tunnel")

    if controller_type != "pt":

        ocp.constraints.lh = np.array(constraints_config["lh"])
        ocp.constraints.uh = np.array(constraints_config["uh"])
        ocp.constraints.lh_e = np.array(constraints_config["lh_e"])
        ocp.constraints.uh_e = np.array(constraints_config["uh_e"])

        # add slacks
        ocp.constraints.idxsh = np.array([0])
        ocp.constraints.idxsh_e = np.array([0])

        Z = np.diag(constraints_config["Z"])
        z = np.array(constraints_config["z"])
        Z_e = np.diag(constraints_config["Z_e"])
        z_e = np.diag(constraints_config["z_e"])

        ocp.cost.Zl = Z
        ocp.cost.Zu = Z
        ocp.cost.Zl_e = Z_e
        ocp.cost.Zu_e = Z_e
        ocp.cost.zl = z
        ocp.cost.zu = z
        ocp.cost.zl_e = z_e
        ocp.cost.zu_e = z_e
    else:
        print("No constraints (path tracking controller)")
        ocp.constraints.lh = np.array([-ACADOS_INFTY])
        ocp.constraints.uh = np.array([ACADOS_INFTY])
        ocp.constraints.lh_e = np.array([-ACADOS_INFTY])
        ocp.constraints.uh_e = np.array([ACADOS_INFTY])
