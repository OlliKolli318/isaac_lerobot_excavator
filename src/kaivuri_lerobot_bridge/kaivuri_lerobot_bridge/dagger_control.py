"""ROS-independent HIL ownership/phase protocol. Host time is only a lease."""
PHASES = {"idle": 0, "autonomous": 1, "paused": 2, "correcting": 3, "off": 4, "armed": 5}


def decode_control(value):
    session, generation, phase = str(value).split(":")
    session, generation = int(session), int(generation)
    if not 0 < session < 2**52 or not 0 <= generation < 2**31 or phase not in PHASES:
        raise ValueError("Invalid DAgger control state")
    return session, generation, phase


class DaggerLease:
    def __init__(self, timeout_s=2.0):
        self.timeout_s = timeout_s
        self.state = ""
        self.last_seen = -float("inf")
        self.retired = set()

    def accept(self, value, now):
        session, generation, phase = decode_control(value)
        if session in self.retired:
            return False
        if self.state:
            old_session, old_generation, old_phase = decode_control(self.state)
            if session != old_session:
                if now - self.last_seen <= self.timeout_s and old_phase != "off":
                    return False
                self.retired.add(old_session)
            elif generation < old_generation or (generation == old_generation and value != self.state):
                return False
            elif generation == old_generation and now - self.last_seen > self.timeout_s:
                # A delayed heartbeat must never restart motion after a timeout.
                return False
        self.state, self.last_seen = value, now
        return True

    def effective(self, now):
        if not self.state:
            return ""
        session, generation, phase = decode_control(self.state)
        if phase == "off":
            return ""
        if now - self.last_seen > self.timeout_s:
            # Keep ownership and hold until a new generation is requested.
            return f"{session}:{generation}:idle"
        return self.state
