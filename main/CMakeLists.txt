idf_component_register(
    SRCS 
        "main.c"
        "trot_gait.c"
        "walk_gait.c"
        "creep_gait.c"
        "crawl_gait.c"
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
