idf_component_register(
    SRCS 
        "main.c"
        # Optional local utilities
        "util/sts3032_config.c"
        
        # Optional: Uncomment to include additional examples
        # "examples/example_modes.c"
        
    INCLUDE_DIRS 
        "."
        "util"
        "examples"
    
    REQUIRES
        sts3032
)
