"""
 Bismillahirahmanirahim
 To Do :
    - config MQTT
    - do calibrate
    - training model
    - testing model
"""

import paho.mqtt.client as mqtt
import time
import datetime
import csv
import numpy as np

# MQTT setup
broker_host = "telemedicine.co.id"
broker_port = 49560
mqtt_topic_data_acc = "FALTO_01/sensor/acc"
mqtt_topic_data_gyro = "FALTO_01/sensor/gyro"
mqtt_topic_callibration = "FALTO_01/sensor/callib"
mqtt_topic_callibration_gyro = "FALTO_01/sensor/callib/gyro"
mqtt_topic_callibration_acc = "FALTO_01/sensor/callib/acc"


# for log file
dt = datetime.datetime.now()
logfile = 'fall_prediction-%s-%s-%s.csv' % (dt.day, dt.month, dt.year)

# csvwrite
def write_tocsv(data) :
    with open(logfile, "a") as output_file:
        writer = csv.writer(output_file, delimiter=',', lineterminator='\r')
        writer.writerow(data)

"""
MQTT PART
"""
#define callback
count_incoming = 1
join_data_testing = np.array([], dtype=float)
join_data_training = np.array([], dtype=float)
def on_message(client, userdata, message):
    """
    Every data incoming from broker will call this function, implement all of logic here
    :param client:
    :param userdata:
    :param message:
    :return:
    """
    global join_data_testing , count_incoming , join_data_training
    print "message topic=", message.topic, " - qos=", message.qos, " - flag=", message.retain
    receivedMessage = str(message.payload.decode("utf-8"))
    print "received message = ", receivedMessage
    signal = receivedMessage.split(':')
    # print(signal)
    write_tocsv(signal)  # write signal to CSV

    features = convert_data(signal)
    if (message.topic == mqtt_topic_data_gyro or message.topic == mqtt_topic_data_acc):
        print("Testing....")
        join_data_testing = np.hstack((join_data_testing, features))

        if (count_incoming == 2):
            print("One records... acc + gyro")
            print("Data testing...")
            print(join_data_testing)
            print("Length : " + str(len(join_data_testing)))
            count_incoming = 0  # reset count incoming
            join_data_testing = np.array([], dtype=float)  # reset join_data

    elif (message.topic == mqtt_topic_callibration_acc or message.topic == mqtt_topic_callibration_gyro):
        print("Training...")
        join_data_training = np.hstack((join_data_training, features))

        if (count_incoming == 2):
            print("One records... acc + gyro")
            print("Data Training... ")
            print(join_data_training)
            print("Length : " + str(len(join_data_training)))
            count_incoming = 0  # reset count incoming
            join_data_training = np.array([], dtype=float)  # reset join_data

    # outer
    count_incoming = count_incoming + 1 # count for 2 incoming data

def convert_data(raw_data):
    # testing features
    features = np.array([], dtype=float)
    # convert to float
    for x in range(len(raw_data) - 1):
        # print(float(signal[x]))
        features = np.hstack((features, float(raw_data[x])))

    return features
def on_connect(client, userdata, flags, rc):
    """
    call when mqtt connect
    :param client:
    :param userdata:
    :param flags:
    :param rc:
    :return:
    """
    print("Connected with result code "+ str(rc))
    # subscribe the topic "ekg/device1/signal"
    # subTopic = "ekg/+/signal"  # + is wildcard for all string to that level
    # subTopic = "rhythm/PPG004/ppg"
    # define topic
    # subTopic = "ppg/signal"
    global clientMQTT
    print "Subscribe topic ", mqtt_topic_data_acc
    clientMQTT.subscribe(mqtt_topic_data_acc)
    print "Subscribe topic ", mqtt_topic_data_gyro
    clientMQTT.subscribe(mqtt_topic_data_gyro)
    print "Subscribe topic ", mqtt_topic_callibration_acc
    clientMQTT.subscribe(mqtt_topic_callibration_acc)
    print "Subscribe topic ", mqtt_topic_callibration_gyro
    clientMQTT.subscribe(mqtt_topic_callibration_gyro)

def main():
    # mqtt instance
    print("Server starting...")
    print("IP Broker : " + broker_host +":"+ str(broker_port))
    # create client
    global clientMQTT
    clientMQTT = mqtt.Client("server-fal")

    # set callback
    clientMQTT.on_message = on_message
    clientMQTT.on_connect = on_connect

    # connection established
    clientMQTT.connect(broker_host, broker_port)  # connect to broker

    clientMQTT.loop_forever()  # loop forever

if __name__ == "__main__":
    main()
