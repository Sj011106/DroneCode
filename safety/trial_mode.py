# =====================================================================================
# safety/trial_mode.py  -  the scored hold can NEVER hand navigation back to GPS   (OWNER: Ayush)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The rules say GNSS (GPS) may help during setup but must NOT aid the scored hold
#   (Grand Final package Stage 1 Rev 1.3, p.2 and p.7 Q14). Saumya's vision_hold.py has
#   FALLBACK_TO_GPS = True: if vision is LOST, or when the program stops, it asks the
#   autopilot to switch back to GPS. Only the --no-gps-fallback flag turned that off, so
#   one forgotten flag = a possibly spoofed GPS takes over the scored hold.
#   TRIAL MODE (the default) makes that impossible in two independent ways:
#     1. it sets --no-gps-fallback for you, so Saumya's own logic never asks for GPS;
#     2. a GUARD sits on the radio link and drops every "use GPS" command, whoever sends
#        it (set_source, command, or a raw MAVLink packet), and counts it in the log.
#   Practice runs keep working: GPS can still be RECORDED by ArduPilot as a referee
#   (tools/truth_drift.py reads it from the .bin log afterwards). Only --practice turns
#   trial mode off, and it prints a loud banner every time.
#
# HOW IT WORKS (technical)
#   - Allowed: MAV_CMD_SET_EKF_SOURCE_SET with source set 2 (our vision). Everything else
#     that could select GPS is dropped: source set 1 or 3, COMMAND_INT with that command,
#     writes to EK3_SRC* / AHRS_GPS_USE / RCx_OPTION, and RC override packets (an RC
#     channel can be an "EKF source set" switch, RCx_OPTION = 90).
#   - The guard wraps the link INSTANCE (instance attributes shadow the class methods), so
#     Saumya's DroneLink class is not edited. set_source is checked BEFORE it records
#     pending_source, so a late ACK can never mark GPS as active.
#   - The wrappers never raise and never take the link's send lock (no deadlock, no crash
#     of the hold loop). Cost: one integer compare per command (commands are ~1 per s).
#
# WHERE IT PLUGS IN
#   vision_hold.py:  trial = trial_mode.from_args(args)   right after parse_args
#                    trial.guard(link)                     right after the link is created
#   config.TRIAL_MODE is the on/off switch (True = trial mode unless --practice).
#   preflight.py reports whether trial mode is ON (NO-GO if not).
#
# REFERENCES
#   - Grand Final Participant Technical Package, Stage 1 Rev 1.3: p.2 ("The scored hold
#     must operate without GNSS aiding"), p.7 Q14.
#   - MAV_CMD_SET_EKF_SOURCE_SET (42007): https://mavlink.io/en/messages/common.html#MAV_CMD_SET_EKF_SOURCE_SET
#   - ArduPilot EKF source sets and RCx_OPTION 90:
#     https://ardupilot.org/copter/docs/common-ekf-sources.html
#     https://ardupilot.org/copter/docs/common-auxiliary-functions.html
# =====================================================================================

import time

import config

MAV_CMD_SET_EKF_SOURCE_SET = 42007   # MAVLink command id (common.xml)
VISION_SOURCE_SET = 2                # the only source set trial mode allows (= link.drone_link.SRC_VISION)
BLOCKED_PARAM_PREFIXES = ("EK3_SRC", "AHRS_GPS_USE", "GPS_TYPE")   # + any RCx_OPTION (checked below)


def _param_name(param_id):
    if isinstance(param_id, (bytes, bytearray)):
        param_id = param_id.decode(errors="ignore")
    return str(param_id).rstrip("\x00").upper()


def blocked_param(param_id):
    """True if writing this parameter could make the EKF use GPS (or put it on a switch)."""
    name = _param_name(param_id)
    if name.startswith(BLOCKED_PARAM_PREFIXES):
        return True
    return name.startswith("RC") and name.endswith("_OPTION")


class TrialMode:
    """Decides trial vs practice, and installs the guard on a link."""

    def __init__(self, active, reason):
        self.active = active
        self.reason = reason
        self.blocked = []                 # (time, what) for every command we dropped
        self.allowed_vision = 0           # how many "use vision" requests went through
        self._last_print = 0.0

    # ----- 1. the flag: Saumya's fallback logic never asks for GPS -----
    def apply_to_args(self, args):
        if self.active:
            args.no_gps_fallback = True
        return args

    def banner(self):
        if self.active:
            return f"TRIAL MODE ON ({self.reason}): GPS fallback impossible, GPS commands are blocked."
        return ("!" * 78 + "\nPRACTICE MODE (--practice): GPS fallback ALLOWED. NEVER use this for the "
                "scored trial.\n" + "!" * 78)

    # ----- 2. the guard on the link -----
    def _block(self, what):
        now = time.time()
        self.blocked.append((now, what))
        if now - self._last_print > 1.0:                  # at most one line per second
            print(f"TRIAL MODE: blocked '{what}' (GNSS must not aid the scored hold); "
                  f"{len(self.blocked)} blocked so far")
            self._last_print = now

    def guard(self, link):
        """Wrap this link object so no GPS-selecting command can leave it. Returns link."""
        if not self.active:
            return link
        trial = self
        # Only wrap what this link has (test fakes may have no command(); replay has no radio).
        orig_set_source = getattr(link, "set_source", None)
        if orig_set_source is not None:
            def set_source(n):
                if int(n) != VISION_SOURCE_SET:
                    trial._block(f"set_source({n})")
                    return None
                trial.allowed_vision += 1
                return orig_set_source(n)

            link.set_source = set_source

        orig_command = getattr(link, "command", None)
        if orig_command is not None:
            def command(cmd, *params, **kw):
                p1 = params[0] if params else kw.get("p1", 0)
                if int(cmd) == MAV_CMD_SET_EKF_SOURCE_SET and int(p1) != VISION_SOURCE_SET:
                    trial._block(f"command(SET_EKF_SOURCE_SET, {p1})")
                    return None
                return orig_command(cmd, *params, **kw)

            link.command = command

        mav = getattr(getattr(link, "conn", None), "mav", None)
        if mav is not None:
            self._guard_mav(mav)
        return link

    def _guard_mav(self, mav):
        trial = self

        def wrap(name, check):
            orig = getattr(mav, name, None)
            if orig is None:
                return

            def guarded(*a, **kw):
                try:
                    what = check(a, kw)
                except Exception:                         # odd arguments: let the original decide
                    what = None
                if what:
                    trial._block(what)
                    return None
                return orig(*a, **kw)

            setattr(mav, name, guarded)

        def long_check(a, kw):      # command_long_send(target_sys, target_comp, command, confirmation, p1..p7)
            cmd = a[2] if len(a) > 2 else kw.get("command")
            p1 = a[4] if len(a) > 4 else kw.get("param1", 0)
            if int(cmd) == MAV_CMD_SET_EKF_SOURCE_SET and int(p1) != VISION_SOURCE_SET:
                return f"COMMAND_LONG SET_EKF_SOURCE_SET {p1}"
            return None

        def int_check(a, kw):       # command_int_send(ts, tc, frame, command, current, autocontinue, p1..)
            cmd = a[3] if len(a) > 3 else kw.get("command")
            p1 = a[6] if len(a) > 6 else kw.get("param1", 0)
            if int(cmd) == MAV_CMD_SET_EKF_SOURCE_SET and int(p1) != VISION_SOURCE_SET:
                return f"COMMAND_INT SET_EKF_SOURCE_SET {p1}"
            return None

        def param_check(a, kw):     # param_set_send(ts, tc, param_id, value, type)
            pid = a[2] if len(a) > 2 else kw.get("param_id", b"")
            return f"PARAM_SET {_param_name(pid)}" if blocked_param(pid) else None

        wrap("command_long_send", long_check)
        wrap("command_int_send", int_check)
        wrap("param_set_send", param_check)
        wrap("rc_channels_override_send", lambda a, kw: "RC_CHANNELS_OVERRIDE")

    def summary(self):
        if not self.active:
            return "Practice mode: GPS fallback was allowed."
        return (f"Trial mode: {len(self.blocked)} GPS command(s) blocked, "
                f"{self.allowed_vision} 'use vision' request(s) allowed.")


def decide(practice_flag, config_value=None):
    """Trial mode unless config.TRIAL_MODE is False or --practice is given."""
    on = config.TRIAL_MODE if config_value is None else config_value
    if practice_flag:
        return TrialMode(False, "--practice")
    if not on:
        return TrialMode(False, "config.TRIAL_MODE = False")
    return TrialMode(True, "default")


def from_args(args):
    """Called by vision_hold.py right after parse_args. Prints the banner."""
    trial = decide(getattr(args, "practice", False))
    trial.apply_to_args(args)
    print(trial.banner())
    return trial
