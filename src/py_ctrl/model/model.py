from dataclasses import dataclass
import json
from typing import List, Optional, Dict
from model.operation import Operation, Transition
from predicates.state import State
import predicates.guards
import predicates.actions
from predicates.guards import AlwaysTrue, Guard, And
from py_ctrl.predicates.guards import AlwaysFalse

@dataclass
class Model(object):
    initial_state: State
    operations: Dict[str, Operation]
    transitions: List[Transition]

    def __post_init__(self):
        ops = {o: "i" for o in self.operations}
        self.initial_state = self.initial_state.next(**ops)

g = predicates.guards.from_str
a = predicates.actions.from_str

# variable domains
robot_poses = ['hcpos1', 'hcpos2', 'above_table']
cylinder_poses = ['hcpos1', 'hcpos2']
tcp_frames = ['suction_cup_1', 'suction_cup_2']

def the_model() -> Model:

    initial_state = State(
        # control variables
        robot_run = False,   # trigger action when true. Change to false and then to true to trigger again
        robot_command = 'move_j',   # move_j, move_l, pick, place
        robot_velocity = 0.5,
        robot_acceleration = 0.3,
        robot_goal_frame = 'unknown',   # where to go with the tool tcp
        robot_tcp_frame = 'suction_cup_1', # the tool tcp to use
        gesture = 'unknown',

        bool_to_plc_1 = False,
        bool_to_plc_2 = False,
        bool_to_plc_3 = False,
        bool_to_plc_4 = False,
        bool_to_plc_5 = False,
        int_to_plc_1 = 0,
        int_to_plc_2 = 0,
        int_to_plc_3 = 0,
        int_to_plc_4 = 0,
        int_to_plc_5 = 0,

        goal_as_string = "cyl_at_hcpos2"
        replan = False,

        aruco_run = False,

        # measured variables
        robot_state = "initial",  # "exec", "done", "failed" 
        robot_pose = 'unknown',

        replanned = False,

        bool_from_plc_1 = False,
        bool_from_plc_2 = False,
        bool_from_plc_3 = False,
        bool_from_plc_4 = False,
        bool_from_plc_5 = False,
        int_from_plc_1 = 0,
        int_from_plc_2 = 0,
        int_from_plc_3 = 0,
        int_from_plc_4 = 0,
        int_from_plc_5 = 0,

        aruco_done = False,

        #estimated
        suction_cup_1_occ = False, # If a suction cup is occupied or not
        suction_cup_2_occ = False,
        cyl_at_hcpos1 = True,
        cyl_at_hcpos2 = False,

        arucos_locked = False,
    )

    ops = {}

    # for each pose, maybe you need to have a separate operaion
    # or use the domain lists?

    # move to above_table
    for p in robot_poses:
        ops[f"move_to_{p}"] = Operation(
            name = f"move_to_{p}",
            precondition = Transition("pre", 
                g(f"!robot_run && robot_pose != {p}"), 
                a(f"robot_command = move_j, robot_run, robot_goal_frame = {p}")),
            postcondition = Transition("post", 
                g(f"robot_state == done"), 
                a(f"!robot_run, robot_pose <- {p}")),
            effects = (),
            to_run = Transition.default()
        )

    # move to poses
    for p in cylinder_poses:
        ops[f"move_to_{p}"] = Operation(
            name = f"move_to_{p}",
            precondition = Transition("pre", 
                g(f"!robot_run && robot_pose == above_table"), 
                a(f"robot_command = move_j, robot_run, robot_goal_frame = {p}")),
            postcondition = Transition("post", 
                g(f"robot_state == done"), 
                a(f"!robot_run, robot_pose <- {p}")),
            effects = (),
            to_run = Transition.default()
        )

    for p in cylinder_poses:
        for cup in ["suction_cup_1", "suction_cup_2"]:
            ops[f"pick_at_{p}_with_{cup}"] = Operation(
                name = f"pick_at_{p}_with_{cup}",
                precondition = Transition("pre", 
                    g(f"(robot_pose == {p}) && !{cup}_occ && cyl_at_{p}"), 
                    a(f"robot_command = pick, robot_tcp_frame = {cup}, robot_run")),
                postcondition = Transition("post", 
                    g(f"robot_state == done"), 
                    a(f"!robot_run, {cup}_occ, !cyl_at_{p}")),
                effects = (),
                to_run = Transition.default()
            )

            ops[f"place_at_{p}_with_{cup}"] = Operation(
                name = f"place_at_{p}_with_{cup}",
                precondition = Transition("pre", 
                    g(f"(robot_pose == {p}) && {cup}_occ && !cyl_at_{p}"), 
                    a(f"robot_command = place, robot_tcp_frame = {cup}, robot_run")),
                postcondition = Transition("post", 
                    g(f"robot_state == done"), 
                    a(f"!robot_run, {cup}_occ, cyl_at_{p}")),
                effects = (),
                to_run = Transition.default()
            )
    
    ops[f"lock_arucos"]= Operation(
        name=f"lock_arucos",
        precondition=Transition("pre", g(f"!arucos_locked && robot_pos == camera"), a("lock_run")),
        postcondition=Transition("post", g(f"lock_done"), a("!lock_run, arucos_locked")),
        effects= (),
        to_run = Transition.default()
    )

    # To be used to run "free" transitions. Not implemented in the runner though, so you have to do that
    transitions: List[Transition] = []

    return Model(
        initial_state,
        ops,
        transitions
    )

def from_goal_to_goal(state: State) -> Guard:
    """
    Create a goal predicate 
    """
    goal: str = state.get("goal_as_string")
    if goal != "":
        return g(goal)
    
    return AlwaysFalse()