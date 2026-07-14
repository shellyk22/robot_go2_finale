from unitree_sdk2py.idl.unitree_go.msg.dds_ import UwbState_



class UwbStateManager:
    def __init__(self):
        self.remote_state = UwbState_(
            version=[0, 0],
            channel=0,
            joy_mode=0,
            orientation_est=0.0,
            pitch_est=0.0,
            distance_est=0.0,
            yaw_est=0.0,
            tag_roll=0.0,
            tag_pitch=0.0,
            tag_yaw=0.0,
            base_roll=0.0,
            base_pitch=0.0,
            base_yaw=0.0,
            joystick=[0.0, 0.0],
            error_state=0,
            buttons=0,
            enabled_from_app=0,
        )

    def update_state(self, msg: UwbState_):
        self.remote_state = msg
