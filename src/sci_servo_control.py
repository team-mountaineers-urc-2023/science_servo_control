#!/usr/bin/env python3

from enum import IntEnum
import time
from threading import Thread

import rospy

from payload_can.payload_lib import Payload
from dynamixel_controller.dynamixel_control import Dynamixel_Controller, OpModes
from science_servo_control.srv import FreeMoveDrum, FreeMoveDrumRequest, FreeMoveDrumResponse, \
                                      LowerLinearActuator, LowerLinearActuatorRequest, LowerLinearActuatorResponse, \
                                      MixSample, MixSampleRequest, MixSampleResponse, \
                                      MoveCuvette, MoveCuvetteRequest, MoveCuvetteResponse, \
                                      MoveLinearActuator, MoveLinearActuatorRequest, MoveLinearActuatorResponse, \
                                      MovePump, MovePumpRequest, MovePumpResponse, \
                                      PreMixDrum, PreMixDrumRequest, PreMixDrumResponse, \
                                      PreSealDrum, PreSealDrumRequest, PreSealDrumResponse, \
                                      RaiseLinearActuator, RaiseLinearActuatorRequest, RaiseLinearActuatorResponse, \
                                      ReadDrumPosition, ReadDrumPositionRequest, ReadDrumPositionResponse, \
                                      RebootDrum, RebootDrumRequest, RebootDrumResponse, \
                                      ScoopSample, ScoopSampleRequest, ScoopSampleResponse, \
                                      SealDrum, SealDrumRequest, SealDrumResponse, \
                                      SpinCentrifuge, SpinCentrifugeRequest, SpinCentrifugeResponse, \
                                      WriteGPIO, WriteGPIORequest, WriteGPIOResponse

# current working setup
U2D2_DEV_NAME = "/dev/urc/mtc/sci_u2d2"
DXL_BAUD = 9600
DXL_IDS = [1,2,3,4,7]

CENTRIFUGE_ID = 7
MAX_CENTRIFUGE_GOAL = 125_000

SCOOP_TIMEOUT        = 5   # in sec
MOVE_CUVETTE_TIMEOUT = 20  # in sec
CENTRIFUGE_TIMEOUT    = 120 # in sec

STOP_PUMP_PPM_SPEED = 90

class LinearActuatorPos(IntEnum):
    TOP = 20
    MIDDLE = 90
    BOTTOM = 180

class ScoopPos(IntEnum):
    PRESCOOP = 300
    PREMIX = -2500
    MIX1 = -2800
    MIX2 = -2200
    POSTSCOOP = -3000
    PRESEALED = -1450
    SEALED = -2023

def wait_seconds(duration):
    end_time = time.time() + duration
    while True:
        time_remaining = end_time - time.time()
        if time_remaining <= 0:
            return
        elif time_remaining <= 0.1:
            time.sleep(0.001)
        elif time_remaining <= 0.5:
            time.sleep(0.01)
        elif time_remaining <= 1.5:
            time.sleep(0.1)
        else:
            time.sleep(1)

### main

def main():
    Sci_Servo_Control().loop()

class Sci_Servo_Control:
    def __init__(self):

        rospy.init_node("sci_servos")

        # connect to ROS
        linear_actuator_service = rospy.get_param("~linear_actuator_service")
        pump_service = rospy.get_param("~pump_service")
        scoop_sample_service = rospy.get_param("~scoop_sample_service")
        mix_sample_service = rospy.get_param("~mix_sample_service")
        seal_drum_service = rospy.get_param("~seal_drum_service")
        free_move_drum_service = rospy.get_param("~free_move_drum_service")
        move_cuvette_service = rospy.get_param("~move_cuvette_service")
        spin_centrifuge_service = rospy.get_param("~spin_centrifuge_service")
        write_gpio_service = rospy.get_param("~write_gpio_service")
        read_drum_position_service = rospy.get_param("~read_drum_position_service")
        reboot_drum_service = rospy.get_param("~reboot_drum_service")
        pre_seal_drum_service = rospy.get_param("~pre_seal_drum_service")
        pre_mix_drum_service = rospy.get_param("~pre_mix_drum_service")
        raise_linear_actuator_service = rospy.get_param("~raise_linear_actuator_service")
        lower_linear_actuator_service = rospy.get_param("~lower_linear_actuator_service")

        self.linear_actuator_srv = rospy.Service(linear_actuator_service, MoveLinearActuator, self.linear_actuator_callback)
        self.pump_srv = rospy.Service(pump_service, MovePump, self.pump_callback)
        self.scoop_sample_service = rospy.Service(scoop_sample_service, ScoopSample, self.scoop_sample_callback)
        self.mix_sample_service = rospy.Service(mix_sample_service, MixSample, self.mix_sample_callback)
        self.seal_drum_service = rospy.Service(seal_drum_service, SealDrum, self.seal_drum_callback)
        self.free_move_drum_service = rospy.Service(free_move_drum_service, FreeMoveDrum, self.free_move_drum_callback)
        self.move_cuvette_service = rospy.Service(move_cuvette_service, MoveCuvette, self.move_cuvette_callback)
        self.spin_centrifuge_service = rospy.Service(spin_centrifuge_service, SpinCentrifuge, self.spin_centrifuge_callback)
        self.write_gpio_service = rospy.Service(write_gpio_service, WriteGPIO, self.write_gpio_callback)
        self.read_drum_position_service = rospy.Service(read_drum_position_service, ReadDrumPosition, self.read_drum_position_callback)
        self.read_drum_position_srv = rospy.Service(reboot_drum_service, RebootDrum, self.reboot_drum_callback)
        self.pre_seal_drum_service = rospy.Service(pre_seal_drum_service, PreSealDrum, self.pre_seal_drum_callback)
        self.pre_mix_drum_service = rospy.Service(pre_mix_drum_service, PreMixDrum, self.pre_mix_drum_callback)
        self.raise_linear_actuator_service = rospy.Service(raise_linear_actuator_service, RaiseLinearActuator, self.raise_linear_actuator_callback)
        self.lower_linear_actuator_service = rospy.Service(lower_linear_actuator_service, LowerLinearActuator, self.lower_linear_actuator_callback)

        # payload object for controlling linear actuators and pumps
        self.payload = Payload()

        # dynamixel controller object for controlling scoops and cetrifuge
        self.dxl_controller = Dynamixel_Controller(U2D2_DEV_NAME, DXL_BAUD, DXL_IDS)
        self.dxl_controller.set_op_mode_all(OpModes.EXTENDED_POS_CONTROL)
        self.dxl_controller.enable_torque_all()

    ### local functions

    def run_pump(self, pump_id, ppm_speed, duration):
        if pump_id == 4: # this is the GPIO relay pump
            self.payload.GPIO_write(1, ppm_speed) # THis is the second item in the GPIO list in the pico (board.GP7)
            if ppm_speed != 0:
                wait_seconds(duration)
                self.payload.GPIO_write(1, 0)
        else:
            print(ppm_speed)
            self.payload.PUMP_write(pump_id, ppm_speed)
            if ppm_speed != STOP_PUMP_PPM_SPEED:
                wait_seconds(duration)
                self.payload.PUMP_write(pump_id, STOP_PUMP_PPM_SPEED)

    def wait_until_dxl_in_pos(self, dxl_id: int, timeout_period: int) -> None:
        timeout = time.time() + timeout_period

        while(not self.dxl_controller.at_goal_pos(dxl_id)):
            if time.time() >= timeout:
                # at this point, dynamixel is probably straining to reach a position that it cannot
                # grab its current position and set that as the goal position so it stops straining
                new_goal_pos = self.dxl_controller.get_curr_pos(dxl_id)
                self.dxl_controller.set_goal_pos(dxl_id, new_goal_pos)
                # also break, just to be safe
                break

    ### callbacks

    def linear_actuator_callback(self, req: MoveLinearActuatorRequest) -> MoveLinearActuatorResponse:
        response = MoveLinearActuatorResponse()

        self.payload.SERVO_write(req.actuator_id, req.pwm_pos)

        return response

    def pump_callback(self, req: MovePumpRequest) -> MovePumpResponse:
        response = MovePumpResponse()

        pump_thread = Thread(target=self.run_pump, args=(req.pump_id, req.ppm_speed, req.duration))
        pump_thread.start()

        return response

    def scoop_sample_callback(self, req: ScoopSampleRequest) -> ScoopSampleResponse:
        response = ScoopSampleResponse()

        self.dxl_controller.set_goal_pos(req.drum_id, ScoopPos.PRESCOOP)
        self.wait_until_dxl_in_pos(req.drum_id, SCOOP_TIMEOUT)

        self.dxl_controller.set_goal_pos(req.drum_id, ScoopPos.POSTSCOOP)
        self.wait_until_dxl_in_pos(req.drum_id, SCOOP_TIMEOUT)

        self.dxl_controller.set_goal_pos(req.drum_id, ScoopPos.PRESCOOP)
        self.wait_until_dxl_in_pos(req.drum_id, SCOOP_TIMEOUT)

        return response

    def mix_sample_callback(self, req: MixSampleRequest) -> MixSampleResponse:
        response = MixSampleResponse()

        # TODO maybe allow for this to be controlled by a param
        for i in range(5):
            self.dxl_controller.set_goal_pos(req.drum_id, ScoopPos.MIX1)
            self.wait_until_dxl_in_pos(req.drum_id, SCOOP_TIMEOUT)

            self.dxl_controller.set_goal_pos(req.drum_id, ScoopPos.MIX2)
            self.wait_until_dxl_in_pos(req.drum_id, SCOOP_TIMEOUT)
        
        self.dxl_controller.set_goal_pos(req.drum_id, ScoopPos.PREMIX)
        self.wait_until_dxl_in_pos(req.drum_id, SCOOP_TIMEOUT)

        return response

    def seal_drum_callback(self, req: SealDrumRequest) -> SealDrumResponse:
        response = SealDrumResponse()

        self.dxl_controller.set_goal_pos(req.drum_id, ScoopPos.PRESEALED)
        self.wait_until_dxl_in_pos(req.drum_id, SCOOP_TIMEOUT)

        self.dxl_controller.set_goal_pos(req.drum_id, ScoopPos.SEALED)
        self.wait_until_dxl_in_pos(req.drum_id, SCOOP_TIMEOUT)

        return response

    def free_move_drum_callback(self, req: FreeMoveDrumRequest) -> FreeMoveDrumResponse:
        response = FreeMoveDrumResponse()
       
        # no need to validate that the dynamixel actually makes it to goal pos
        # since this function will be controlled by a knob
        self.dxl_controller.set_goal_pos(req.drum_id, req.drum_pos)

        return response

    def move_cuvette_callback(self, req: MoveCuvetteRequest) -> MoveCuvetteResponse:
        response = MoveCuvetteResponse()

        angle_for_spec = 825 + int(292.57 * (req.cuvette_num-1))

        self.dxl_controller.set_goal_pos(CENTRIFUGE_ID, angle_for_spec)
        self.wait_until_dxl_in_pos(CENTRIFUGE_ID, MOVE_CUVETTE_TIMEOUT)
        
        return response

    def spin_centrifuge_callback(self, req: SpinCentrifugeRequest) -> SpinCentrifugeResponse:
        response = SpinCentrifugeResponse()

        start_pos = self.dxl_controller.get_curr_pos(CENTRIFUGE_ID)
        self.dxl_controller.set_goal_pos(CENTRIFUGE_ID, MAX_CENTRIFUGE_GOAL)
        self.wait_until_dxl_in_pos(CENTRIFUGE_ID, CENTRIFUGE_TIMEOUT)

        self.dxl_controller.set_goal_pos(CENTRIFUGE_ID, start_pos)
        self.wait_until_dxl_in_pos(CENTRIFUGE_ID, CENTRIFUGE_TIMEOUT)

        return response

    def write_gpio_callback(self, req: WriteGPIORequest) -> WriteGPIOResponse:
        response = WriteGPIOResponse()

        self.payload.GPIO_write(req.pin_id, req.pin_value)

        return response

    def read_drum_position_callback(self, req: ReadDrumPositionRequest) -> ReadDrumPositionResponse:
        response = ReadDrumPositionResponse()

        response.drum_pos = self.dxl_controller.get_curr_pos(req.drum_id)

        return response
    
    def reboot_drum_callback(self, req: RebootDrumRequest) -> RebootDrumResponse:
        response = RebootDrumResponse()

        self.dxl_controller.reboot(req.drum_id)

        return response
    
    def pre_seal_drum_callback(self, req: PreSealDrumRequest) -> PreSealDrumResponse:
        response = PreSealDrumResponse()

        self.dxl_controller.set_goal_pos(req.drum_id, ScoopPos.PRESEALED)
        self.wait_until_dxl_in_pos(req.drum_id, SCOOP_TIMEOUT)

        return response

    def pre_mix_drum_callback(self, req: PreMixDrumRequest) -> PreMixDrumResponse:
        response = PreMixDrumResponse()

        self.dxl_controller.set_goal_pos(req.drum_id, ScoopPos.PREMIX)
        self.wait_until_dxl_in_pos(req.drum_id, SCOOP_TIMEOUT)

        return response

    def raise_linear_actuator_callback(self, req: RaiseLinearActuatorRequest) -> RaiseLinearActuatorResponse:
        response = RaiseLinearActuatorResponse()

        self.payload.SERVO_write(req.actuator_id, LinearActuatorPos.TOP)

        return response

    def lower_linear_actuator_callback(self, req: LowerLinearActuatorRequest) -> LowerLinearActuatorResponse:
        response = LowerLinearActuatorResponse()

        self.payload.SERVO_write(req.actuator_id, LinearActuatorPos.MIDDLE)

        return response

    ### loop

    def loop(self):
        rospy.spin()

if __name__ == "__main__":
    main()
