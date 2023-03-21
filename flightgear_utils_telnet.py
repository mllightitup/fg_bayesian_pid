class FGUtils:
    def __init__(self, props_conn):
        self.props_conn = props_conn

    def set_aileron(self, value):
        self.props_conn.set_prop('/controls/flight/aileron', value)

    def set_elevator(self, value):
        self.props_conn.set_prop("/controls/flight/elevator", value)

    def set_rudder(self, value):
        self.props_conn.set_prop("/controls/flight/rudder", value)

    def set_altitude(self, value):
        self.props_conn.set_prop('/position/altitude-ft', value)

    def set_throttle(self, value):
        self.props_conn.set_prop("/controls/engines/current-engine/throttle", value)

    def set_vertical_speed(self, value):
        self.props_conn.set_prop("/velocities/vertical-speed-fps", value)

    def set_ground_speed(self, value):
        self.props_conn.set_prop("/velocities/groundspeed-kt", value)

    def set_pitch(self, value):
        self.props_conn.set_prop("/orientation/pitch-deg", value)

    def set_roll(self, value):
        self.props_conn.set_prop("/orientation/roll-deg", value)

    def set_heading(self, value):
        self.props_conn.set_prop("/orientation/heading-deg", value)

    def set_pitch_model(self, value):
        self.props_conn.set_prop("/orientation/model/pitch-deg", value)

    def set_roll_model(self, value):
        self.props_conn.set_prop("/orientation/model/roll-deg", value)

    def set_heading_model(self, value):
        self.props_conn.set_prop("/orientation/model/heading-deg", value)

    def get_pitch(self):
        return self.props_conn.get_prop("/orientation/pitch-deg")

    def get_elevator(self):
        return self.props_conn.get_prop("/controls/flight/elevator")

    def get_altitude_above_sea(self):
        return self.props_conn.get_prop('/position/altitude-ft')

    def get_altitude_above_ground(self):
        return self.props_conn.get_prop('/position/altitude-ft') * 3.28084

    def get_heading(self):
        return self.props_conn.get_prop("/orientation/heading-deg")

    def get_vertical_speed(self):
        return self.props_conn.get_prop("/velocities/vertical-speed-fps")

    def get_roll_deg(self):
        return self.props_conn.get_prop("/orientation/roll-deg")

    def hold_aileron(self):
        j = 1 / 90
        roll_deg = self.get_roll_deg()
        aileron = -j * roll_deg
        self.set_aileron(max(-1, min(aileron, 1)))

    # def plot_create(self, n, dt, list1, list2, list3, list4):
    #     if n % 255 == 0:
    #         fig, axes = plt.subplots(nrows=1, ncols=4, figsize=(10, 3))
    #         axes[0].plot(list1)
    #         axes[0].set_title('Altitude')
    #
    #         axes[1].plot(list2)
    #         axes[1].set_title('vs_error')
    #
    #         axes[2].plot(list3)
    #         axes[2].set_title('control_signal_vs')
    #
    #         axes[3].plot(list4)
    #         axes[3].set_title('elevator_signal')
    #
    #         fig.suptitle(f"{n} ticks = {round((dt * n), 2)}s -> 1 tick = ~{round(dt, 2)}s")
    #         plt.show()