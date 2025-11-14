"""
LittleFS Patch Script

This script updates the Adafruit nRF52 Arduino framework's LittleFS library
from version 1.6 (used in Adafruit 1.7.0) to version 1.7.2.

"""

from pathlib import Path

Import("env")  # pylint: disable=undefined-variable

try:
  SCRIPT_DIR = Path(__file__).resolve().parent
except NameError:
  SCRIPT_DIR = Path(env.subst("$PROJECT_DIR")) / "extra_scripts"
PATCH_DIR = SCRIPT_DIR / "littlefs_patch"


def _copy_if_different(source: Path, target: Path) -> bool:
  if not source.exists():
    return False

  src_data = source.read_bytes()

  if not target.exists():
    target.write_bytes(src_data)
    return True

  dst_data = target.read_bytes()
  if src_data != dst_data:
    target.write_bytes(src_data)
    return True

  return False


def _apply_littlefs_patch(target, source, env):  # pylint: disable=unused-argument
  framework_path = env.get("PLATFORMFW_DIR")
  if not framework_path:
    framework_path = env.PioPlatform().get_package_dir("framework-arduinoadafruitnrf52")

  if not framework_path:
    print("LittleFS patch: framework directory not found")
    return

  framework_dir = Path(framework_path)

  targets = {
      PATCH_DIR / "lfs.c": framework_dir / "libraries" / "Adafruit_LittleFS" / "src" / "littlefs" / "lfs.c",
      PATCH_DIR / "lfs.h": framework_dir / "libraries" / "Adafruit_LittleFS" / "src" / "littlefs" / "lfs.h",
  }

  updated = False
  for source, target in targets.items():
    if not source.exists():
      print(f"LittleFS patch: source missing {source}")
      continue

    changed = _copy_if_different(source, target)
    status = "updated" if changed else "unchanged"
    print(f"LittleFS patch: {source.name} {status}")
    updated |= changed

  if updated:
    print("LittleFS patch: applied updates")
  else:
    print("LittleFS patch: already up to date")


littlefs_action = env.VerboseAction(_apply_littlefs_patch, "")
env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", littlefs_action)
_apply_littlefs_patch(None, None, env)
