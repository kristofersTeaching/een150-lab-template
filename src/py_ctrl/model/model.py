from dataclasses import dataclass
import json
from typing import List, Optional, Dict
from model.operation import Operation, Transition
from predicates.state import State
import predicates.guards
import predicates.actions
from predicates.guards import AlwaysTrue, Guard, And

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

positions = {f"pos{i}": 'empty' for i in range(1,10)}
positions.update(input = 'empty', output = 'empty')


def the_model() -> Model:


    initial_state = State(
        # control variables
        robot_run = False,   # trigger action when true. Change to false and then to true to trigger again
        robot_command = 'move_j',   # move_j, move_l, 
        robot_velocity = 1.0,
        robot_acceleration = 1.0,
        robot_goal_frame = 'pos1',   # where to go with the tool tcp
        robot_tcp_frame = 'svt_tcp', # the tool tcp to use
        
        marker_run = False,
        marker_parent = 'pos1',
        marker_name = 'cube1',

        bool_to_plc1 = False,
        bool_to_plc2 = False,
        bool_to_plc3 = False,
        bool_to_plc4 = False,
        bool_to_plc5 = False,
        int_to_plc1 = 0,
        int_to_plc2 = 0,
        int_to_plc3 = 0,
        int_to_plc4 = 0,
        int_to_plc5 = 0,

        # measured variables
        robot_state = "initial",  # "exec", "done", "failed" 
        robot_act_pos = 'unknown',
        marker_done = False,
        new_marker_name = 'cube_name',

        #estimated
        gripper = 'empty',
        robot_pos = 'unknown',
        **{f"in_{p}": v for p, v in positions.items()},

    )


    ops = {}

    ops[f"add_cube"] = Operation(
        name = f"add_cube",
        precondition = Transition("pre", 
            g(f"in_input == empty"), 
            a(f"marker_run, marker_parent <- input, marker_name <- generate")),
        postcondition = Transition("post", 
            g(f"marker_done"), 
            a(f"!marker_run, in_input <- new_marker_name")),
        effects = (),
    )

    ops[f"remove_cube"] = Operation(
        name = f"remove_cube",
        precondition = Transition("pre", 
            g(f"in_output != empty"), 
            a(f"marker_run, marker_parent <- remove, marker_name <- in_output")),
        postcondition = Transition("post", 
            g(f"marker_done"), 
            a(f"!marker_run, in_output <- empty")),
        effects = (),
    )

    for p in positions.keys():
        ops[f"to_{p}"] = Operation(
            name = f"to_{p}",
            precondition = Transition("pre", 
                g(f"!robot_run && robot_pos != {p} && ((in_{p} == empty && gripper != empty) || (in_{p} != empty && gripper == empty)) "), 
                a(f"robot_run, robot_goal_frame = {p}")),
            postcondition = Transition("post", 
                g(f"robot_state == done"), 
                a(f"!robot_run, robot_pos <- {p}")),
            effects = (),
        )

        ops[f"pick_at_{p}"] = Operation(
            name = f"pick_at_{p}",
            precondition = Transition("pre", 
                g(f"(robot_pos == {p}) && (in_{p} != empty) && (gripper == empty)"), 
                a(f"marker_run, marker_parent <- svt_tcp, marker_name <- in_{p}")),
            postcondition = Transition("post", 
                g(f"marker_done"), 
                a(f"!marker_run, gripper <- in_{p}, in_{p} <- empty")),
            effects = (),
        )

        ops[f"place_at_{p}"] = Operation(
            name = f"place_at_{p}",
            precondition = Transition("pre", 
                g(f"(robot_pos == {p}) && (in_{p} == empty) && (gripper != empty)"), 
                a(f"marker_run, marker_parent <- {p}, marker_name <- gripper")),
            postcondition = Transition("post", 
                g(f"marker_done"), 
                a(f"!marker_run, in_{p} <- gripper,  gripper <- empty")),
            effects = (),
        )


    # To be used to run "free" transitions. Not implemented in the runner though, so you have to do that
    transitions: List[Transition] = []


    


    return Model(
        initial_state,
        ops,
        transitions
    )


def from_goal_to_goal(goal: str) -> Guard:
    """
    Create a goal predicate 
    """

    dict = json.loads(goal)
    try:
        xs: set[str] = set(dict['positions'])
        cubes_at: list[Guard] = []
        cubes_at.append(g(f"gripper == empty"))
        for p in positions.keys():
            if p in xs:
                cubes_at.append(g(f"in_{p} != empty"))
            else:
                cubes_at.append(g(f"in_{p} == empty"))

        return And(*cubes_at)

    except KeyError:
        return AlwaysTrue()





