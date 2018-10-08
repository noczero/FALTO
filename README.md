# FALTO

Device Setting <br />

stream data sampling : <br/>
mqtt_topic_data_acc = "FALTO_01/sensor/acc" <br/>
mqtt_topic_data_gyro = "FALTO_01/sensor/gyro" <br/>

send command to do callibration <br/>
mqtt_topic_callibration = "FALTO_01/sensor/callib" <br/>
payload command :
- walking
- stand
- sit
- terlentang

stream data on callibration <br/>
mqtt_topic_callibration_gyro = "FALTO_01/sensor/callib/gyro" <br/>
mqtt_topic_callibration_acc = "FALTO_01/sensor/callib/acc" <br/>

stream data realtime : <br/>
mqtt_topic_callibration_acc_unit = "FALTO_01/sensor/callib/acc/unit <br/>
mqtt_topic_callibration_gyro_unit = "FALTO_01/sensor/callib/gyro/unit <br/>

Sampling 20Hz

# mqtt_topic_data_acc output :
Format : acc X : acc Y : acc Z : RMSS(ROOT MEAN SUM SQUARE) acc : next data .... <br/>
out : 

0.37:-0.40:-22.03:12.73:0.53:-0.24:-21.82:12.60:0.52:-0.48:-21.81:12.60:0.46:-0.43:-21.95:12.68:0.37:-0.37:-21.97:12.69:0.46:-0.38:-21.92:12.66:0.32:-0.47:-22.00:12.71:0.46:-0.40:-22.04:12.73:0.46:-0.43:-22.00:12.70:0.45:-0.43:-21.90:12.65:0.51:-0.49:-22.09:12.76:0.41:-0.41:-21.93:12.67:0.34:-0.46:-22.02:12.72:0.40:-0.33:-22.05:12.73:0.36:-0.32:-21.87:12.63:0.39:-0.40:-21.75:12.56:0.55:-0.38:-21.85:12.62:0.55:-0.37:-21.87:12.64:0.45:-0.41:-21.99:12.70:0.41:-0.27:-21.81:12.59:

# mqtt_topic_data_gyro output :
Format : gyro X : gyro Y : gyro Z : RMSS(ROOT MEAN SUM SQUARE) gyro : next data .... <br/>
out :

0.02:0.09:0.05:0.06:-0.00:0.09:0.03:0.06:-0.03:0.09:0.10:0.08:-0.01:0.08:0.06:0.06:-0.01:0.09:0.09:0.07:-0.02:0.08:0.04:0.05:-0.03:0.07:0.07:0.06:-0.03:0.06:0.10:0.07:-0.01:0.08:0.04:0.05:-0.03:0.06:0.10:0.07:-0.01:0.09:0.09:0.08:-0.04:0.09:0.08:0.07:-0.04:0.05:0.08:0.06:0.00:0.08:0.08:0.06:-0.01:0.09:0.07:0.06:0.01:0.12:0.06:0.08:-0.01:0.09:0.09:0.07:-0.03:0.08:0.04:0.05:0.01:0.09:0.05:0.06:-0.01:0.10:0.06:0.07:
