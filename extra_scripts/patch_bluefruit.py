from pathlib import Path


def _patch_ble_advertising(source: Path) -> None:
    text = source.read_text()

    # Already patched?
    if "static ble_gap_adv_data_t gap_adv = {" in text and "sd_ble_gap_adv_stop" not in text:
        return

    stop_block = (
        "  // stop first if current running since we may change advertising data/params\n"
        "  if (_running) {\n"
        "    sd_ble_gap_adv_stop(_hdl);\n"
        "  }\n\n"
    )

    replacement_block = (
        "  // gap_adv long-live is required by SD v6\n"
        "  static ble_gap_adv_data_t gap_adv = {\n"
        "      .adv_data      = { .p_data = _data, .len = _count },\n"
        "      .scan_rsp_data = { .p_data = Bluefruit.ScanResponse.getData(), .len = Bluefruit.ScanResponse.count() }\n"
        "  };\n"
    )

    original_block = (
        "  // gap_adv long-live is required by SD v6\n"
        "  static ble_gap_adv_data_t gap_adv;\n"
        "  gap_adv.adv_data.p_data = _data;\n"
        "  gap_adv.adv_data.len = _count;\n"
        "  gap_adv.scan_rsp_data.p_data = Bluefruit.ScanResponse.getData();\n"
        "  gap_adv.scan_rsp_data.len = Bluefruit.ScanResponse.count();\n"
    )

    updated = False

    if stop_block in text:
        text = text.replace(stop_block, "", 1)
        updated = True

    if original_block in text:
        text = text.replace(original_block, replacement_block, 1)
        updated = True

    if updated:
        source.write_text(text)


def before_build(env, platform):  # pylint: disable=unused-argument
    framework_dir = Path(env.get("PLATFORMFW_DIR", ""))
    if not framework_dir:
        return

    target = framework_dir / "libraries" / "Bluefruit52Lib" / "src" / "BLEAdvertising.cpp"
    if target.exists():
        _patch_ble_advertising(target)


