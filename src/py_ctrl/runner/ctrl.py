from typing import Any, Optional, Tuple, List
import json
from predicates.state import State
from model.model import from_goal_to_goal, the_model, Model
from planner.plan import plan
from model.operation import Operation
from predicates.state import State
from predicates.errors import NotInStateException
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.publisher import Publisher
from ur_tools_msgs.action import URScriptControl
from viz_tools_msgs.srv import ManipulateDynamicMarker
import random


import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

# ---------------------------------------------------------------------------
# ...
# ---------------------------------------------------------------------------


runner_goal: str = "runner_goal"
runner_plan: str = "runner_plan"
step_in_plan: str = "step_in_plan"
plan_status: str = "plan_status"

# publish a goal:

class Runner(Node):
    marker_done: bool = False
    cube_id = 1
    
    def __init__(self):
        super().__init__('the_runner')
        self.model: Model = the_model()
        self.state: State = self.model.initial_state
        self.upd_state(runner_goal, None)
        self.upd_state(runner_plan, None)
        self.upd_state(step_in_plan, None)
        self.upd_state(plan_status, None)

        self.ur_robot_action_client = ActionClient(self, URScriptControl, '/ur_script_controller')
        self.robot_action_goal_handle: Optional[ClientGoalHandle] = None

        ## We will not use the goal topic. Should be defind using the state
        self.create_subscription(
            msg_type = String,
            topic = 'goal',
            callback = self.goal_callback,
            qos_profile = 10)

        self.create_subscription(
            msg_type = String,
            topic = 'set_state',
            callback = self.set_state_callback,
            qos_profile = 10)

        self.pub_state: Publisher = self.create_publisher(
            msg_type=String,
            topic = 'state',
            qos_profile = 10,
        )

        self.create_subscription(
            msg_type = String,
            topic = '/opc_measured',
            callback = self.set_opc_callback,
            qos_profile = 10)

        self.pub_opc: Publisher = self.create_publisher(
            msg_type=String,
            topic = '/opc_command',
            qos_profile = 10,
        )

        self.timer = self.create_timer(0.1, self.ticker)


    def goal_callback(self, msg: String):
        """
        Here the goal comes in from ros and if we do not have a goal, or if the goal is 
        new, it will reset the runner so it will replan.
        """
        goal = self.state.get(runner_goal)
        if goal != msg:
            print(f"We got a new goal: {msg}")
            self.upd_state(runner_goal, msg.data)
            self.upd_state(runner_plan, None)
            self.upd_state(step_in_plan, None)
            self.upd_state(plan_status, None)
    
    def set_state_callback(self, msg: String):
        """
        Here you can send in state changes from outside
        """
        try:
            j = msg.data.replace('\'', '\"')
            kvs: dict[str, Any] = json.loads(j)
            print(f"got a state change: {kvs}")
            self.state = self.state.next(**kvs)
        except TypeError:
            pass
        except json.decoder.JSONDecodeError as e:
            print(f"message is bad: {msg.data}")
            print(e)

    def set_opc_callback(self, msg: String):
        """
        Here the state from te PLC comes in
        """
        try:
            j = msg.data.replace('\'', '\"')
            kvs: dict[str, Any] = json.loads(j)

            fixed = {k.replace("ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.", ""): v for k,v in kvs.items()}
            self.state = self.state.next(**fixed)
        except TypeError:
            pass
        except json.decoder.JSONDecodeError as e:
            print(f"message is bad: {msg.data}")
            print(e)

            

    def send_ur_action_goal(self):
        run: bool = self.state.get('robot_run')
        if not run and self.robot_action_goal_handle is not None:
            self.robot_action_goal_handle.cancel_goal()  # maybe do it async?
            print("Cancel of robot action done")
            self.robot_action_goal_handle = None
            self.upd_state('robot_state', "initial")
        elif not run:
            self.upd_state('robot_state', "initial")
        elif run and self.robot_action_goal_handle is None: 
            print("start action")
            goal_msg = URScriptControl.Goal()
            goal_msg.command = self.state.get('robot_command')
            goal_msg.velocity = self.state.get('robot_velocity')
            goal_msg.acceleration = self.state.get('robot_acceleration')
            goal_msg.goal_feature_name = self.state.get('robot_goal_frame')
            goal_msg.tcp_name = self.state.get('robot_tcp_frame')

            print("waiting action")
            if self.ur_robot_action_client.wait_for_server(2):
                print("done waiting action")
                send_goal_future = self.ur_robot_action_client.send_goal_async(goal_msg)
                send_goal_future.add_done_callback(self.ur_action_goal_response_callback)
            else:
                print("timeout action")


    def ur_action_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.upd_state('robot_state', "failed")
            self.robot_action_goal_handle = None
            return

        self.robot_action_goal_handle = goal_handle
        self.get_logger().info('Goal accepted :)')
        self.upd_state('robot_state', "exec")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.ur_action_get_result_callback)


    def ur_action_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.upd_state('robot_state', "done")
        self.upd_state('robot_act_pos', self.state.get('robot_goal_frame'))
        self.robot_action_goal_handle = None


    def send_marker_service(self):
        run: bool = self.state.get('marker_run')
        if run and not self.marker_done:
            self.marker_done = True
            print("waiting for service")
            service = self.create_client(ManipulateDynamicMarker, "/manipulate_dynamic_marker")
            service.wait_for_service()
            print("Service ready")
            msg = ManipulateDynamicMarker.Request()
            msg.absolute_mesh_path = "/ros/ia_ros_meshes/cube.stl"
            msg.primitive_type = 0
            msg.color.a = 1.0
            msg.color.r = random.uniform(0.3, 0.9)
            msg.color.g = random.uniform(0.3, 0.9)
            msg.color.b = random.uniform(0.3, 0.9)
            msg.scale.x = 0.01
            msg.scale.y = 0.01
            msg.scale.z = 0.01

            name = self.state.get('marker_name')
            if name == 'generate':
                msg.parent_id = self.state.get('marker_parent')
                msg.child_id = f"cube{self.cube_id}"
                self.cube_id += 1
                msg.command = "update"
                self.upd_state('new_marker_name', msg.child_id)
            elif self.state.get('marker_parent') == 'remove':
                msg.parent_id = "in_output"
                msg.child_id = self.state.get('marker_name')
                msg.command = "remove"
            else:
                msg.parent_id = self.state.get('marker_parent')
                msg.child_id = self.state.get('marker_name')
                msg.command = "update"

            resp = service.call_async(msg)
            resp.add_done_callback(self.marked_done_callback)
            
        elif not run and self.marker_done:
            self.marker_done = False

        self.upd_state('marker_done', self.marker_done)
    
    def marked_done_callback(self, future):
        result = future.result().success
        if not result:
            print(f"The marker service did not like the call. Check log in simulator window")
        self.marker_done = result
        self.upd_state('marker_done', self.marker_done)


    def send_to_opc(self):
        kv = {
            "bool_to_plc1": self.state.get("bool_to_plc1"),
            "bool_to_plc2": self.state.get("bool_to_plc2"),
            "bool_to_plc3": self.state.get("bool_to_plc3"),
            "bool_to_plc4": self.state.get("bool_to_plc4"),
            "bool_to_plc5": self.state.get("bool_to_plc5"),
            "int_to_plc1": self.state.get("int_to_plc1"),
            "int_to_plc2": self.state.get("int_to_plc2"),
            "int_to_plc3": self.state.get("int_to_plc3"),
            "int_to_plc4": self.state.get("int_to_plc4"),
            "int_to_plc5": self.state.get("int_to_plc5"),
        }
        kv = {f"ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.{k}": v for k, v in kv.items()}
        j = json.dumps(kv)
        self.pub_opc.publish(String(data = j))


    def upd_state(self, key: str, value):
        self.state = self.state.next(**{key: value})

    def ticker(self):
        g = self.state.get(runner_goal)
        p = self.state.get(runner_plan)
            
        if g and not p and not self.state.get(plan_status):
            # this should be changed so that you instead check variables in the state for the goal. 

            # if we have a new goal, let us replan
            goal = from_goal_to_goal(g)
            print(f"The goal: {goal}")
            new_p = plan(self.state, goal, self.model, 30)
            self.upd_state(runner_plan, new_p)
            print(f"The new goal: {goal}")
            print(f"and computed this plan: {new_p}")

        prev_state = self.state

        # here we call the ticker. Change the pre_start parameter to true when
        # you want to prestart
        self.state = tick_the_runner(self.state, self.model, True)

        if prev_state != self.state:
            print(f"")
            for k, v in self.state.items():
                print(f"{k}: {v}")
            print(f"")
        

        # below, we are publishing the command variables to the simulation via ros
        self.send_ur_action_goal()
        self.send_marker_service()
        state_json = json.dumps(self.state.state)
        self.pub_state.publish(String(data = state_json))
        


def tick_the_runner(state: State, model: Model, pre_start: bool) -> State:
    """
    This function will run the operations based on a plan that are located in the state
    This will just execute one transition at the time
    """

    # Here you can execute the free transitions by checking if they are enabled and then do next on them

    the_plan: list[str] = state.get(runner_plan)    
    if not the_plan:
        return state.next(plan_status="No plan in state")
    
    current_step_in_plan: int = state.get(step_in_plan)
    if not current_step_in_plan:
        # we have not started executing the plan so we start at position 0 in the plan
        current_step_in_plan = 0
        state = state.next(**{step_in_plan: current_step_in_plan})
    
    plan_length = len(the_plan)
    if plan_length <= current_step_in_plan:
        # we are done with the plan and will stop executing and we also
        # reset the current plan so we do not tries to run the same plan again
        return state.next(plan_status="done", runner_plan = None, step_in_plan = None)

    # check what operation we are / should be executing
    current_op_name = the_plan[current_step_in_plan]
    current_op_state: str = state.get(current_op_name)
    current_op: Operation = model.operations[current_op_name]

    next_step = current_step_in_plan + 1

    if current_op_state == "i" and current_op.eval(state): # The operation can be started
        next_state = current_op.start(state)
    elif current_op_state == "i": # the operation should be started but is not enabled
        next_state = state.next(plan_status=f"waiting for op {current_op_name} to be enabled. pre: {current_op.precondition}")
    elif current_op.is_completed(state): # the operation has completed and we can take a step in the plan
        next_state = current_op.complete(state)
        next_state = next_state.next(step_in_plan=next_step, plan_status=f"completing step {current_step_in_plan}")
    elif current_op_state == "e": # the operation is executing, let's check if we can prestart the next
        if not pre_start:
            next_state = state.next(plan_status=f"waiting for op to complete")
        elif plan_length > next_step and model.operations[the_plan[next_step]].eval(state):
            next_state = model.operations[the_plan[next_step]].start(state).next(
                plan_status=f"pre_starting {next_step}"
            )
        else:
            next_state = state
    else:
        next_state = state.next(plan_status="doing nothing")
    
    return next_state



def run():
    rclpy.init()
    runner = Runner()
    rclpy.spin(runner)
    runner.destroy_node()
    rclpy.shutdown()

