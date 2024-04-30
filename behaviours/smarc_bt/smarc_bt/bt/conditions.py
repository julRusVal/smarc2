#!/usr/bin/python3

import enum
from typing import Callable

from py_trees.common import Status
from py_trees.blackboard import Blackboard
from py_trees.behaviour import Behaviour

from .i_has_vehicle_container import HasVehicleContainer
from .i_has_clock import HasClock
from .common import VehicleBehaviour, MissionPlanBehaviour, bool_to_status
from .bb_keys import BBKeys
from ..mission.mission_plan import MissionPlanStates
from ..vehicles.sensor import SensorNames


class C_SensorAllowedSilence(VehicleBehaviour):
    def __init__(self,
                 bt: HasClock,
                 sensor_name: str,
                 allowed_silence_seconds: int,
                 initial_silence_seconds:int = 5):
        name = name = f"{self.__class__.__name__}({sensor_name} (allow {allowed_silence_seconds}s))"
        super().__init__(bt, name)
        self._sensor_name = sensor_name
        self._allowed_silence_seconds = allowed_silence_seconds

        self._first_tick_seconds = None
        self._initial_silence_seconds = initial_silence_seconds

    @property
    def _now(self):
        return self._bt.now_seconds

    def update(self) -> Status:
        if self._first_tick_seconds is None:
            self._first_tick_seconds = self._now
    
        dt_since_first = self._now - self._first_tick_seconds
        if dt_since_first < self._initial_silence_seconds:
            self.feedback_message = f"{dt_since_first:.0f}/{self._initial_silence_seconds} of initial silence."
            return Status.SUCCESS

        sensor = self._bt.vehicle_container.vehicle_state[self._sensor_name]
        
        if sensor.last_update_seconds is None:
            self.feedback_message = f"Sensor has never published!"
            return Status.FAILURE

        
        dt = self._now - sensor.last_update_seconds 
        if dt > self._allowed_silence_seconds:
            self.feedback_message = f"{dt} > {self._allowed_silence_seconds}!"
            return Status.FAILURE
        
        self.feedback_message = f"{dt:.1f}s since last update"
        return Status.SUCCESS
        

        
class C_CheckSensorBool(VehicleBehaviour):
    def __init__(self,
                 bt: HasVehicleContainer,
                 sensor_name: str,
                 sensor_key = 0):
        """
        Returns S if vehicle[sensor_name][sensor_key] == True, F otherwise
        """
        name = name = f"{self.__class__.__name__}({sensor_name}[{sensor_key}])"
        self._sensor_name = sensor_name
        self._sensor_key = sensor_key
        super().__init__(bt, name)

    def update(self) -> Status:
        sensor = self._bt.vehicle_container.vehicle_state[self._sensor_name]
        return bool_to_status(sensor[self._sensor_key])
    



class C_SensorOperatorBlackboard(VehicleBehaviour):
    def __init__(self,
                 bt: HasVehicleContainer,
                 sensor_name: str,
                 operator: Callable,
                 bb_key: enum.Enum,
                 sensor_key = 0):
        """
        Returns S if operator(vehicle[sensor_name][sensor_key], bb[bb_key]) == True
        """
        name = f"C_{sensor_name}[{sensor_key}] {operator.__name__} {bb_key}"
        self._sensor_name = sensor_name
        self._sensor_key = sensor_key
        self._bb_key = bb_key
        self._operator = operator
        super().__init__(bt, name)

    def update(self) -> Status:
        sensor = self._bt.vehicle_container.vehicle_state[self._sensor_name]
        value = sensor[self._sensor_key]
        bb = Blackboard()
        
        if not bb.exists(self._bb_key):
            self.feedback_message = f"Key {self._bb_key} not in BB!"  
            return Status.FAILURE
        
        bb_value = bb.get(self._bb_key)
        bb_value_str = "None"
        if bb_value is not None:
            bb_value_str = f"{bb_value:.2f}"
            
        value_str = "None"
        if value is not None:
            value_str = f"{value:.2f}"

        self.feedback_message = f"{self._operator.__name__}({value_str}, {bb_value_str})"

        if value is None or bb_value is None:
            return Status.FAILURE

        self.feedback_message = f"{self._operator.__name__}({value_str}, {bb_value_str})"
        return bool_to_status(self._operator(value, bb_value))
        
        
class C_NotAborted(VehicleBehaviour):
    def __init__(self, bt: HasVehicleContainer):
        super().__init__(bt)

    def update(self) -> Status:
        if self._bt.vehicle_container.vehicle_state.aborted:
            self.feedback_message = "!! ABORTED !!"
            return Status.FAILURE
        
        return Status.SUCCESS


class C_CheckMissionPlanState(MissionPlanBehaviour):
    def __init__(self, expected_state: MissionPlanStates):
        self._expected_state = expected_state
        name = f"{self.__class__.__name__}({self._expected_state})"
        super().__init__(name)
        self._bb = Blackboard()
        

    def update(self) -> Status:
        self.feedback_message = ""
        plan = self._get_plan()
        if plan is None: return Status.FAILURE

        if plan.state != self._expected_state:
            self.feedback_message = f"Expected:{self._expected_state} found:{plan.state}"
            return Status.FAILURE

        return Status.SUCCESS
    

class C_MissionTimeoutOK(MissionPlanBehaviour):
    def __init__(self):
        name = f"{self.__class__.__name__}"
        super().__init__(name)
        self._bb = Blackboard()

    def update(self) -> Status:
        self.feedback_message = ""
        plan = self._get_plan()
        if plan is None: return Status.SUCCESS
        
        self.feedback_message = f"({plan.seconds_to_timeout}) to timeout"
        if plan.timeout_reached: 
            return Status.FAILURE
        return Status.SUCCESS