# system_init.py

import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import UwbState_
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

from uwb_state_manager import UwbStateManager
from uwb_button_monitor import UwbButtonMonitor

from follow_controller import FollowConfig, FollowController


class SystemInit:
    """
    Centralized initializer for all subsystems:
      - Unitree communications
      - UWB state manager
      - Sport + Obstacle Avoid clients
      - FollowController thread
    """

    def __init__(self, config):
        """
        config: your AppConfig object (constants only)
        """
        self.cfg = config

    # ------------------------------------------------------------
    # UNITREE (UWB + SPORT + AVOID + BUTTON MONITOR)
    # ------------------------------------------------------------
    def init_unitree(self, estop_callback=None):
        print("[INIT] Initializing Unitree SDK...")

        ChannelFactoryInitialize(0)

        # UWB / state manager
        state_manager = UwbStateManager()

        # אם לא הועברה פונקציית עצירה מבחוץ, נשתמש בהדפסה כברירת מחדל
        if estop_callback is None:
            estop_callback = lambda: print("[UWB] Shutdown button pressed.")

        button_monitor = UwbButtonMonitor(
            state_manager, 
            estop_callback
        )

        uwb_sub = ChannelSubscriber("rt/uwbstate", UwbState_)
        uwb_sub.Init(button_monitor.get_callback(), 10)

        # Sport + obstacle clients
        sport = SportClient()
        avoid = ObstaclesAvoidClient()
        avoid.Init()
        sport.Init()
        avoid.UseRemoteCommandFromApi(True)
        avoid.SwitchSet(True)

        print("[INIT] Unitree communication established.")

        # Return handles
        return state_manager, sport, avoid

    # ------------------------------------------------------------
    # FOLLOW CONTROLLER THREAD
    # ------------------------------------------------------------
    def init_follower(self, state_manager, avoid, behavior, stop_event):
        print("[INIT] Starting FollowController thread...")

        follow_cfg = FollowConfig(
            SMOOTH_ALPHA=self.cfg.SMOOTH_ALPHA,
            MAX_VX=self.cfg.MAX_VX,
            MAX_WZ=self.cfg.MAX_WZ,
            FOLLOW_DT=self.cfg.FOLLOW_DT,
            DEAD_BAND_D=self.cfg.DEAD_BAND_D,
            DIST_SLOWDOWN=self.cfg.DIST_SLOWDOWN,
            DEAD_BAND_O=self.cfg.DEAD_BAND_O,
            SLOWDOWN_ANGLE=self.cfg.SLOWDOWN_ANGLE,
            MAX_VX_FOLLOW=self.cfg.MAX_VX_FOLLOW,
            MAX_WZ_FOLLOW=self.cfg.MAX_WZ_FOLLOW,
        )

        follower = FollowController(state_manager, avoid, behavior, follow_cfg)
        follower.start(stop_event, daemon=True)

        print("[INIT] FollowController is running.")
        return follower