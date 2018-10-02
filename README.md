# FALTO

Device Setting <br />
#define mqtt_server "192.168.1.2" <br />
#define mqtt_port 1883 <br />
#define device_name "FALTO_DEV01" <br />
#define mqtt_topic_data_acc "FALTO_01/sensor/acc" <br />
#define mqtt_topic_data_gyro "FALTO_01/sensor/gyro" <br />


# mqtt_topic_data_acc output :
Format : acc X : acc Y : acc Z : RMS(ROOT MEAN SQUARE) acc, next data .... <br/>
out : 

2.41:0.50:-21.79:12.66,2.49:0.60:-21.65:12.58,2.37:0.56:-22.03:12.80,2.34:0.40:-21.77:12.64,2.32:0.63:-21.82:12.67,2.47:0.68:-22.00:12.78,2.51:0.65:-21.91:12.73,2.49:0.44:-21.81:12.67,2.41:0.43:-21.89:12.71,2.28:0.49:-21.83:12.68,2.31:0.52:-21.91:12.72,2.37:0.62:-21.75:12.63,2.30:0.54:-21.75:12.63,2.48:0.41:-21.96:12.76,2.38:0.69:-22.01:12.79,2.48:0.50:-21.82:12.68,2.41:0.61:-21.73:12.63,2.33:0.52:-21.73:12.62,2.32:0.44:-21.69:12.60,2.30:0.50:-21.80:12.66,

# mqtt_topic_data_gyro output :
Format : gyro X : gyro Y : gyro Z : RMS(ROOT MEAN SQUARE) gyro, next data .... <br/>
out :

-0.03:0.07:0.06,0.00:0.05:0.07,-0.03:0.05:0.09,0.01:0.05:0.06,-0.01:0.07:0.09,-0.01:0.05:0.09,-0.01:0.09:0.05,-0.00:0.06:0.06,-0.01:0.08:0.10,-0.00:0.05:0.05,-0.04:0.04:0.07,0.00:0.08:0.08,-0.03:0.06:0.08,-0.01:0.04:0.09,-0.03:0.07:0.09,-0.03:0.05:0.09,0.03:0.08:0.06,-0.01:0.07:0.10,-0.00:0.06:0.09,-0.00:0.07:0.08,
