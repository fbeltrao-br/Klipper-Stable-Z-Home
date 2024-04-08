# Repeats homing on the Z axis until the resulting stepper position has
# fully stabilized within tolerance, and optionally runs a specific Gcode
# sequence before each attempt.
#
# Copyright (C) 2021 Matthew Lloyd <github@matthewlloyd.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging


class StableZHome:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.before_homing_gcode = gcode_macro.load_template(config, 'gcode')
        self.default_max_retries = config.getint("retries", 20, minval=0)
        self.default_retry_tolerance = \
            config.getfloat("retry_tolerance", 1 / 400., above=1/1000.)
        self.default_window = config.getint("window", 4, minval=3)
        self.default_z_hop = config.getfloat("z_hop", 0., minval=0.)
        self.allow_home = config.getboolean('allow_home', False)
        # Register STABLE_Z_HOME command
        self.gcode.register_command(
            'STABLE_Z_HOME', self.cmd_STABLE_Z_HOME,
            desc=self.cmd_STABLE_Z_HOME_help)
    cmd_STABLE_Z_HOME_help = (
        "Repeatedly home Z until the Z stepper position stabilizes")
    def cmd_STABLE_Z_HOME(self, gcmd):
        max_retries = gcmd.get_int('RETRIES', self.default_max_retries,
                                   minval=0)
        retry_tolerance = gcmd.get_float('RETRY_TOLERANCE',
                                         self.default_retry_tolerance,
                                         minval=1/1000.)
        window = gcmd.get_int('WINDOW', self.default_window, minval=3)
        z_hop = gcmd.get_float('Z_HOP', self.default_z_hop, minval=0.)

        toolhead = self.printer.lookup_object('toolhead', None)
        if toolhead is None:
            raise gcmd.error("Printer not ready")
        kin = toolhead.get_kinematics()

        # Check X and Y are homed first.
        curtime = self.printer.get_reactor().monotonic()
        homed_axes = kin.get_status(curtime)['homed_axes']
        if 'x' not in homed_axes or 'y' not in homed_axes:
            if not self.allow_home:
                raise gcmd.error("Must home X and Y axes first")
            else:
                try:
                    self.gcode.run_script_from_command('G28')
                except Exception:
                    logging.exception("Exception trying to home")
                    raise self.gcode.error('Homing failed')

        steppers = kin.get_steppers()
        stepper = None
        for s in steppers:
            if s.get_name().startswith('stepper_z'):
                stepper = s
                break
        if stepper is None:
            raise gcmd.error("No Z steppers found")

        self.gcode.respond_info(
            'Stable Z home: %.4f tolerance, window %d, %d max retries\n'
            % (retry_tolerance, window, max_retries))

        mcu_z_readings = []
        retries = 1
        retry_tolerance += 1e-4  # allow for floating point rounding errors
        
        try:
            self.gcode.run_script_from_command(
            self.before_homing_gcode.render())
        except Exception:
            logging.exception("Exception running pre-home script")
            raise self.gcode.error('Pre-home Gcode failed')
        
        while retries <= max_retries:
            self.gcode.run_script_from_command('G28 Z')

            mcu_position_offset = -stepper.mcu_to_commanded_position(0)
            mcu_pos = stepper.get_commanded_position() + mcu_position_offset
            mcu_z_readings.append(mcu_pos)
            mcu_z_readings = mcu_z_readings[-window:]
            if len(mcu_z_readings) == window:
                window_range = max(mcu_z_readings) - min(mcu_z_readings)
            else:
                window_range = None

            window_range_str = \
                '%.4f' % (window_range,) if window_range is not None else '-'
            self.gcode.respond_info(
                'Retry %d: %s position %.4f, window range %s\n'
                % (retries, stepper.get_name(), mcu_pos, window_range_str))
            try:
                self.gcode.run_script_from_command('G0 Z' + str(z_hop))
            except Exception:
                logging.exception("Exception trying to z_hop")
                raise self.gcode.error('Z hop failed')

            if window_range is not None and window_range <= retry_tolerance:
                self.gcode.respond_info('Succeeded\n')
                break

            retries += 1

        if retries > max_retries:
            raise self.gcode.error('Max retries exceeded\n')


def load_config(config):
    return StableZHome(config)
