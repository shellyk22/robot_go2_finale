# app_config.py
import math

class AppConfig:
    # -------------------- Motion Control --------------------
    MAX_VX = 1.0      # (במקור היה 0.40) מגדיל את המהירות הכללית המותרת
    MAX_WZ = 0.96    
    K_WZ = 1.2
    K_VX_FWD = 0.8
    K_VX_BACK = 0.4
    SMOOTH_ALPHA = 0.2
    FOLLOW_DT = 0.04

    # -------------------- Follow controller (UWB) --------------------
    DEAD_BAND_D = 1.2
    DIST_SLOWDOWN = 1.0
    DEAD_BAND_O = 0.20
    SLOWDOWN_ANGLE = math.radians(60)
    MAX_VX_FOLLOW = 2.0   # (במקור היה 0.9) מאפשר הליכה מהירה מאוד / ריצה קלה
    MAX_WZ_FOLLOW = 0.96