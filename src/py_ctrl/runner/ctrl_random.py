from typing import Any, Optional, Tuple, List
import json
from predicates.state import State
from model.model import from_goal_to_goal, the_model, Model
from planner.plan import plan
from model.operation import Operation
from predicates.state import State
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
        Here the goal comes in from ros and if we do not have a goal, or if the goal is 
        new, it will reset the runner so it will replan.
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
            print(f"The markser service did not like the call. Check log in simluator window")
        self.marker_done = result
        self.upd_state('marker_done', self.marker_done)



    def upd_state(self, key: str, value):
        self.state = self.state.next(**{key: value})

    def ticker(self):
        g = self.state.get(runner_goal)
        p = self.state.get(runner_plan)
            
        if g and not p and not self.state.get(plan_status):
            # if we have a new goal, let us replan
            goal = from_goal_to_goal(g)
            new_p = plan(self.state, goal, self.model, 30)
            self.upd_state(runner_plan, new_p)
            print(f"The new goal: {goal}")
            print(f"and computed this plan: {new_p}")

        prev_state = self.state

        # here we call the ticker. Change the pre_start parameter to true when
        # you want to prestart
        self.state = tick_the_random_runner(self.state, self.model)

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
        


def tick_the_random_runner(state: State, model: Model) -> State:
    """
    This function will run the operations based on a plan that are located in the state
    This will just execute one transition at the time
    """

    running_ops: List[Operation] = [o for name, o in model.operations.items() if state.get(name) == "e"]
    next_state = state

    if not running_ops:
        enabled_ops = [o for _, o in model.operations.items() if o.precondition.eval(state)]
        if not enabled_ops:
            print("No operations are enabled or running in this state!")
            return state
        
        o = random.choice(enabled_ops)
        next_state = o.start(state)
        print(f"Operation {o.name} started!")

    else:
        ops_can_complete = [o for o in running_ops if o.postcondition.eval(state)]
        if ops_can_complete:
            next_state = state
            for o in ops_can_complete:
                next_state = o.complete(next_state)
                print(f"Operation {o.name} completed!")

    return next_state



def run():
    rclpy.init()
    runner = Runner()
    rclpy.spin(runner)
    runner.destroy_node()
    rclpy.shutdown()

