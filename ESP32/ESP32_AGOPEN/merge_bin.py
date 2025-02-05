Import("env")

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.bin",
    env.VerboseAction(" ".join([
        "python",  # Itt biztosítjuk, hogy Python fusson
        "$PROJECT_PACKAGES_DIR/tool-esptoolpy/esptool.py",
        "--chip", "esp32",
        "merge_bin", "-o", "$BUILD_DIR/esp32_firmware.bin",
        "--flash_mode", "dio",
        "--flash_freq", "40m",
        "--flash_size", "4MB",
        "0x1000", "$BUILD_DIR/bootloader.bin",
        "0x8000", "$BUILD_DIR/partitions.bin",
        "0xe000", "$PROJECT_PACKAGES_DIR/framework-arduinoespressif32/tools/partitions/boot_app0.bin",
        "0x10000", "$BUILD_DIR/${PROGNAME}.bin"
    ]), "Merging firmware into single bin file")
)