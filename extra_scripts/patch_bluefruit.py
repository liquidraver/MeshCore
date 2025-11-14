"""
Bluefruit BLE Advertising Patch Script

This script removes the unnecessary "stop first if current running" block from
BLEAdvertising.cpp in the Adafruit nRF52 Arduino framework.
SoftDevice v6 API's sd_ble_gap_adv_set_configure() is designed to update advertising data/parameters while advertising is active, so no need for unneccessary stops.

"""

from pathlib import Path

Import("env")  # pylint: disable=undefined-variable


def _patch_ble_advertising(source: Path) -> None:
    text = source.read_text()

    if "sd_ble_gap_adv_stop" not in text:
        return

    stop_block = (
        "  // stop first if current running since we may change advertising data/params\n"
        "  if (_running) {\n"
        "    sd_ble_gap_adv_stop(_hdl);\n"
        "  }\n\n"
    )

    if stop_block in text:
        text = text.replace(stop_block, "", 1)
        source.write_text(text)


def _apply_bluefruit_patch(target, source, env):  # pylint: disable=unused-argument
    framework_path = env.get("PLATFORMFW_DIR")
    if not framework_path:
        framework_path = env.PioPlatform().get_package_dir("framework-arduinoadafruitnrf52")

    if not framework_path:
        print("Bluefruit patch: framework directory not found")
        return

    framework_dir = Path(framework_path)
    target = framework_dir / "libraries" / "Bluefruit52Lib" / "src" / "BLEAdvertising.cpp"
    if target.exists():
        before = target.read_text()
        _patch_ble_advertising(target)
        after = target.read_text()
        if before != after:
            print("Bluefruit patch: applied updates")
        else:
            print("Bluefruit patch: already up to date")
    else:
        print("Bluefruit patch: target file not found")


bluefruit_action = env.VerboseAction(_apply_bluefruit_patch, "")
env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", bluefruit_action)
_apply_bluefruit_patch(None, None, env)


