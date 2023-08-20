import aerosandbox as asb
import aerosandbox.numpy as np
from aerosandbox.tools import units as u
from . import cessna152

from aerosandbox.weights.mass_properties_of_shapes import mass_properties_from_radius_of_gyration

def run(ros_node, h_0, v_0):

    ros_node.get_logger().info(f"Starting simulation...")

    mass_props = mass_properties_from_radius_of_gyration(
        mass=1151.8 * u.lbm,
        radius_of_gyration_x=2,
        radius_of_gyration_y=3,
        radius_of_gyration_z=3,
    )
    mass_props.x_cg = 1

    ### Initialize the problem
    opti = asb.Opti()

    ### Define time. Note that the horizon length is unknown.
    time_final_guess = 100
    time = np.cosspace(
        0,
        opti.variable(init_guess=time_final_guess, log_transform=True),
        100
    )
    N = np.length(time)

    time_guess = np.linspace(0, time_final_guess, N)

    ### Create a dynamics instance
    init_state = {
        "x_e"  : 0,
        "z_e"  : -h_0,  # 1 km altitude
        "speed": v_0 * u.knot,
        "gamma": 0,
    }

    dyn = asb.DynamicsPointMass2DSpeedGamma(
        mass_props=mass_props,
        x_e=opti.variable(init_state["speed"] * time_guess),
        z_e=opti.variable(np.linspace(init_state["z_e"], 0, N)),
        speed=opti.variable(init_guess=init_state["speed"], n_vars=N),
        gamma=opti.variable(init_guess=0, n_vars=N, lower_bound=-np.pi / 2, upper_bound=np.pi / 2),
        alpha=opti.variable(init_guess=5, n_vars=N, lower_bound=-5, upper_bound=15),
    )
    # Constrain the initial state
    for k in dyn.state.keys():
        opti.subject_to(
            dyn.state[k][0] == init_state[k]
        )

    ### Add in forces
    dyn.add_gravity_force(g=9.81)

    aero = asb.AeroBuildup(
        airplane=cessna152.airplane,
        op_point=dyn.op_point
    ).run()

    dyn.add_force(
        *aero["F_w"],
        axes="wind"
    )

    ### Constrain the altitude to be above ground at all times
    opti.subject_to(
        dyn.altitude > 0
    )

    ### Finalize the problem
    dyn.constrain_derivatives(opti, time)  # Apply the dynamics constraints created up to this point

    opti.minimize(-dyn.x_e[-1])  # Go as far downrange as you can

    ### Solve it
    sol = opti.solve()

    ### Substitute the optimization variables in the dynamics instance with their solved values (in-place)
    dyn.substitute_solution(sol)

    ros_node.get_logger().info(f"Simulation finished successfully!")
    return (dyn.state['x_e'], dyn.state['z_e'], dyn.state['speed'])