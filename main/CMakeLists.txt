idf_component_register(
    SRCS 
        "main.c"
        "dog/dog_config.c"
        "gaits/trot_gait.c"
        "gaits/walk_gait.c"
        "gaits/creep_gait.c"
        "gaits/crawl_gait.c"
        # Minimal BLE servo control
        "ble/ble_servo.c"
        # Optional local utilities
        "util/sts3032_config.c"
        
        # Optional: Uncomment to include additional examples
        # "examples/example_modes.c"
        
    INCLUDE_DIRS 
        "."
        "dog"
        "gaits"
        "util"
        "examples"
        "ble"
    
    REQUIRES
        sts3032
        bt
        json
        nvs_flash
)
