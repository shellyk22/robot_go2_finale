# uwb_button_monitor.py

class UwbButtonMonitor:
    """
    Monitors the UWB remote buttons and triggers a callback when a 
    specific button is pressed (e.g., Emergency Stop).
    """

    BUTTON_M = 4  
    BUTTON_P = 8

    def __init__(self, state_manager, on_button_press_callback):
        self.state_manager = state_manager
        self.callback = on_button_press_callback
        self._last_button_state = 0

    def get_callback(self):
        def _internal_callback(msg):
            # 1. עדכון המצב הכללי ב-StateManager
            self.state_manager.update_state(msg)
            
            current_buttons = msg.buttons
            
            # --- קוד עזר לבדיקה (DEBUG) ---
            if current_buttons != self._last_button_state and current_buttons != 0:
                print(f"[UWB DEBUG] Button pressed! Value from remote: {current_buttons}")
            # -------------------------------
            
            # 2. בדיקה האם כפתור M או P נלחצו כעת
            # בודקים כל כפתור בנפרד (bitmask)
            m_is_pressed = (current_buttons & self.BUTTON_M) != 0
            p_is_pressed = (current_buttons & self.BUTTON_P) != 0
            
            # בודקים שמצב הכפתור הקודם היה "משוחרר" (כדי למנוע הפעלה כפולה כשהאצבע לחוצה רצוף)
            m_was_not_pressed = (self._last_button_state & self.BUTTON_M) == 0
            p_was_not_pressed = (self._last_button_state & self.BUTTON_P) == 0
            
            # התנאי הסופי: אם M נלחץ עכשיו, *או* P נלחץ עכשיו -> תפעיל את הפונקציה!
            if (m_is_pressed and m_was_not_pressed) or (p_is_pressed and p_was_not_pressed):
                if self.callback:
                    print(f"[UWB CONTROL] Stop triggered by button {'M' if m_is_pressed else 'P'}!")
                    self.callback()
            
            # שמירת המצב האחרון
            self._last_button_state = current_buttons

        return _internal_callback