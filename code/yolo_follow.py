import signal
import time
import threading
import logging

# Project imports
from AppConfig import AppConfig
from system_init import SystemInit

logger = logging.getLogger(__name__)

# -------------------- Globals --------------------
stop_event = threading.Event()

# The behavior dictionary remains because FollowController writes to it and motion_executor reads from it
behavior = {
    "mode": "FOLLOW",  # Always remain in follow mode (UWB)
    "vx": 0.0,
    "vy": 0.0,         # Lateral speed (Strafing) critical for evasion
    "wz": 0.0,
}

# -------------------- Utilities --------------------

def motion_executor(sport_client, behavior_dict, stop_evt, rate_hz=50):
    """
    Dedicated thread to execute motion commands.
    We are using `sport_client` to prevent the AI watchdog from freezing the robot!
    """
    dt = 1.0 / rate_hz
    while not stop_evt.is_set():
        # Reading the latest velocities calculated by the UWB controller in the background
        vx_target = behavior_dict.get("vx", 0.0)
        vy_target = behavior_dict.get("vy", 0.0)  
        wz_target = behavior_dict.get("wz", 0.0)

        try:
            # Sending the motion command directly to the sport layer
            sport_client.Move(vx_target, vy_target, wz_target) 
        except Exception as e:
            pass # Ignoring temporary errors so as not to flood the terminal

        time.sleep(dt)
    
    # Safe stop of the robot upon exit
    try:
        sport_client.Move(0.0, 0.0, 0.0)
    except:
        pass
    print("[MOTION] Executor thread stopped.")

def handle_sigint(signum, frame):
    print("\n[SYS] Ctrl+C detected — stopping...")
    stop_event.set()

def emergency_stop_handler():
    """Triggered by the UWB remote button to stop everything instantly"""
    print("\n[EMERGENCY] UWB Button Pressed! STOPPING ROBOT NOW!")
    stop_event.set()

# -------------------- Main --------------------
def main():



    signal.signal(signal.SIGINT, handle_sigint)

    # =========================================================
    # Experimenter question - executed first thing before any robot initialization!
    # =========================================================
    print("\n" + "="*50)
    subject_num = input("  [SYS] Please enter the subject number (e.g., 1, 2, 3): ").strip()
    if not subject_num:
        subject_num = "default"
    print("="*50 + "\n")

    # Saving the number so the walking controller can pull it and open an appropriate folder
    behavior["subject_num"] = subject_num

    print("[SYS] Initializing startup delay (7 seconds)...")
    time.sleep(7.0)

    # ------------------------------------------------------------
    # Initialize all system components using SystemInit
    # ------------------------------------------------------------
    sys = SystemInit(AppConfig)

    state_manager, sport, avoid = sys.init_unitree(estop_callback=emergency_stop_handler)
    avoid.UseRemoteCommandFromApi(False)
    time.sleep(0.5)

    # =========================================================
    # Set Classic Walk (Trot) Mode Automatically
    # =========================================================
    print("[SYS] Setting Classic Walk (Trot) via code...")
    sport.SwitchGait(2)
    
    # Giving the internal computer time to exit AI mode before sending motion commands
    time.sleep(1.0) 
    print("[SYS] Classic Walk activated successfully.")
    # =========================================================

    # =========================================================
    # Force EXIT from AVOID mode and setup SPORT mode
    # =========================================================
    print("[SYS] Forcing robot out of AVOID/AI mode...")
    
    avoid.UseRemoteCommandFromApi(False)
    avoid.SwitchSet(False)
    time.sleep(0.5)

    print("[SYS] Sending Recovery Stand to clear state...")
    sport.RecoveryStand()
    
    time.sleep(1.5) 

    print("[SYS] Setting Classic Walk (Trot) via code...")
    sport.SwitchGait(2)
    time.sleep(1.0) 
    print("[SYS] Robot is now in SPORT mode and ready.")
    # =========================================================

    # --- Activating the motors thread (Motion Executor) ---
    print("[MOTION] Executor thread started (Using Sport Layer).")
    motion_thread = threading.Thread(
        target=motion_executor,
        args=(sport, behavior, stop_event), 
        daemon=True
    )
    motion_thread.start()

    # --- Activating the walking controller and UWB thread ---
    follower = sys.init_follower(state_manager, avoid, behavior, stop_event)
    follower.stop_event = stop_event

    print("[SYS] All systems initialized. Walking system is running.")
    print("[SYS] Press Ctrl+C in this terminal to stop.\n")

    # -------------------- Main wait loop --------------------
    try:
        # The main loop now only keeps the program alive.
        # All calculation and motion logic is executed asynchronously in the background threads.
        while not stop_event.is_set():
            time.sleep(0.1)
     
    finally:
        stop_event.set()
        # Safe stop and resource release
        try:
            sport.Move(0.0, 0.0, 0.0) 
        except:
            pass
        follower.join(timeout=1.0)
        print("[SYS] Shutdown complete.")

if __name__ == "__main__":
    main()