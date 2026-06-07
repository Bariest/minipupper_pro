idf_component_register(SRCS "ftservo.c"
                             "SCSerail.c"
                             "SCS.c"
                             "SCSCL.c"
                             "SMS_STS.c"
                             "HLS.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_driver_uart esp_driver_gpio freertos log)
