idf_component_register(
    SRCS "st_uld/vl53l5cx_api.c"
         "vl53l5cx_platform.c"
         "vl53l5cx_drv.c"
    INCLUDE_DIRS "." "st_uld"
    REQUIRES driver
)
