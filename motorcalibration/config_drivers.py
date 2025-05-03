import odrive
from odrive.enums import *
from fibre.libfibre import ObjectLostError


ODRIVE_CONFIGS = {
    "20763599524B": {"label": "front_hip", "axis0_cpr": 4000, "axis1_cpr": 2048},
    "2081356E524B": {"label": "front_thigh", "axis0_cpr": 4000, "axis1_cpr": 2000},
    "205B39814D4D": {"label": "front_calf", "axis0_cpr": 4000, "axis1_cpr": 2000},
    "208339814D4D": {"label": "rear_hip", "axis0_cpr": 2000, "axis1_cpr": 2000},
    "345735573033": {"label": "rear_thigh", "axis0_cpr": 2048, "axis1_cpr": 2000},
    "346035533033": {"label": "rear_calf", "axis0_cpr": 4000, "axis1_cpr": 2000},
}

def configure_axis(axis, cpr):
    axis.encoder.config.cpr = cpr

    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    #axis.controller.config.input_filter_bandwidth = 0

    axis.motor.config.current_lim = 27.5
    axis.motor.config.calibration_current = 27.5
    axis.config.calibration_lockin.current = 27.5

    axis.controller.config.vel_limit = 12
    axis.motor.config.pole_pairs = 11
    axis.motor.config.torque_constant = 8.27/340

def main():
    print("Starting ODrive configuration...")

    for serial, config in ODRIVE_CONFIGS.items():
        label = config["label"]

        odrv = odrive.find_any(serial_number=serial, timeout=10)

        # Drive specific config
        odrv.config.dc_max_positive_current = 40.0
        odrv.config.dc_max_negative_current = -8.0

        print(f"Configuring {label} axis0 with CPR {config['axis0_cpr']}")
        configure_axis(odrv.axis0, config["axis0_cpr"])

        print(f"Configuring {label} axis1 with CPR {config['axis1_cpr']}")
        configure_axis(odrv.axis1, config["axis1_cpr"])

        try:
            print(f"Saving configuration for {label} ({serial})...")
            odrv.save_configuration()
        except ObjectLostError:
            pass

        print(f"Configuration saved for {label} ({serial})\n")


if __name__ == '__main__':
    main()