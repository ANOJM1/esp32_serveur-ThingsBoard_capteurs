# Edit following two lines to set component requirements (see docs)
idf_component_register(SRCS "main.c" "sensors.c"  "./STTS751/stts751_reg.c" "./LPS22HH/lps22hh_reg.c" 
"./HTS221/hts221_reg.c"
                      INCLUDE_DIRS "." "STTS751" "." "LPS22HH" "." "HTS221")
