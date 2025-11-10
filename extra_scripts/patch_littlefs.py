from pathlib import Path


def _copy_if_different(source: Path, target: Path) -> bool:
  if not target.exists():
    return False

  src_data = source.read_bytes()
  dst_data = target.read_bytes()
  if src_data == dst_data:
    return False

  target.write_bytes(src_data)
  return True


def before_build(env, platform):  # pylint: disable=unused-argument
  framework_dir = Path(env.get("PLATFORMFW_DIR", ""))
  if not framework_dir:
    return

  script_dir = Path(__file__).parent
  patch_dir = script_dir / "littlefs_patch"

  targets = {
      patch_dir / "lfs.c": framework_dir / "libraries" / "Adafruit_LittleFS" / "src" / "littlefs" / "lfs.c",
      patch_dir / "lfs.h": framework_dir / "libraries" / "Adafruit_LittleFS" / "src" / "littlefs" / "lfs.h",
  }

  updated = False
  for source, target in targets.items():
    if source.exists():
      updated |= _copy_if_different(source, target)

  if updated:
    print("Applied LittleFS patches from meshtastic fork")
