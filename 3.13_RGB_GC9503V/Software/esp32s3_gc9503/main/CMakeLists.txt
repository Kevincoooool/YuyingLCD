set(SOURCES 
            "rgb_lcd_example_main.c"
            "lvgl_demo_ui.c"
            )


idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "."
                    )
