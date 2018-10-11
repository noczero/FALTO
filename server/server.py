"""
 Bismillahirahmanirahim
 To Do :
    - config MQTT
    - do calibrate
    - training model
    - testing model
"""

import paho.mqtt.client as mqtt
import datetime
import csv
import os
import numpy as np

from sklearn.externals import joblib

# MQTT setup
broker_host = "192.168.43.2"
broker_port = 49877
device_name = "FALTO_01"
mqtt_topic_data_acc = device_name  + "/sensor/acc"
mqtt_topic_data_gyro = device_name  + "/sensor/gyro"
mqtt_topic_callibration = device_name + "/sensor/callib"
mqtt_topic_callibration_gyro = device_name  + "/sensor/callib/gyro"
mqtt_topic_callibration_acc = device_name  + "/sensor/callib/acc"
mqtt_topic_result_prob = device_name + "/sensor/p"
mqtt_topic_result = device_name  + "/sensor/n"
mqtt_topic_callibration_ftsts = device_name  + "/sensor/callib/ftsts"

# model name
# 5 agak best
model_name = 'model/knn.joblib.pkl'
model_name_regression = 'model/knn_regression_25.joblib.pkl'

# for log file
dt = datetime.datetime.now()
path = 'dataset/'
logfile = 'fall_prediction-%s-%s-%s.csv' % (dt.day, dt.month, dt.year)
dataset_file_classification= 'fall_dataset_classification-%s-%s-%s-%s.csv' % (dt.day, dt.month, dt.year, dt.second)
dataset_file_regression= 'fall_dataset_regression-%s-%s-%s-%s.csv' % (dt.day, dt.month, dt.year , dt.second)

# csvwrite
def write_tocsv(data , file) :
    print("save to csv...")
    with open(path + file, "a") as output_file:
        writer = csv.writer(output_file, delimiter=',', lineterminator='\r')
        writer.writerow(data)

"""
MQTT PART
"""
#define callback
count_incoming = 1
join_data_testing = np.array([], dtype=float)
join_data_training_str = np.array([] , dtype=str)
label_class = ''
label_regression = 0
classification_status = False

def on_message(client, userdata, message):
    """
    Every data incoming from broker will call this function, implement all of logic here
    :param client:
    :param userdata:
    :param message:
    :return:
    """
    global join_data_testing , count_incoming , join_data_training, join_data_training_str , label_class, clientMQTT, classification_status, label_regression
    print "message topic=", message.topic, " - qos=", message.qos, " - flag=", message.retain
    receivedMessage = str(message.payload.decode("utf-8"))
    print "received message = ", receivedMessage

    if (message.topic == mqtt_topic_callibration):
        if(receivedMessage == "reg"):
            print("Regression model, please input the fall status : 0 - 1")
            # label_regression =
            label_regression = raw_input("Probability : ") # return string
            print("Label Regression : " + label_regression)
            classification_status = False
        else:
            print("Classification model")
            label_class = define_class(receivedMessage) # define class
            print(label_class)
            classification_status = True
    else:
        signal = receivedMessage.split(':')
        # print(signal)
        write_tocsv(signal , logfile)  # write signal to CSV
        features = convert_data(signal) # string to float

        if (message.topic == mqtt_topic_data_gyro or message.topic == mqtt_topic_data_acc):
            print("Testing....")
            join_data_testing = np.hstack((join_data_testing, features))

            if (count_incoming == 2):
                print("One records... acc + gyro")
                print("Data testing...")
                #print(join_data_testing)
                print("Length : " + str(len(join_data_testing)))
                # testing signal

                if(len(join_data_testing) == 160):
                    result = testing_signal(model_name,join_data_testing) # classfication
                    probability = testing_signal(model_name_regression,join_data_testing) # regression
                    # publish
                    clientMQTT.publish(mqtt_topic_result , str(result))
                    clientMQTT.publish(mqtt_topic_result_prob , str(probability))
                    print("Result : " + str(result))
                else:
                    print("Packet not valid...")

                count_incoming = 0  # reset count incoming
                join_data_testing = np.array([], dtype=float)  # reset join_data

        # make data set
        elif (message.topic == mqtt_topic_callibration_acc or message.topic == mqtt_topic_callibration_gyro):
            print("Training...")
            #join_data_training = np.hstack((join_data_training, features))
            join_data_training_str = np.hstack((join_data_training_str , signal))

            if (count_incoming == 2):
                print("One records... acc + gyro")
                print("Data Training... ")

                # check for classify or regression
                if(classification_status):
                    join_data_training_str = np.hstack((join_data_training_str , label_class))
                else:
                    join_data_training_str = np.hstack((join_data_training_str , label_regression))

                join_data_training_str = strip_list_noempty(join_data_training_str) # remove space
                # save to csv
                if (len(join_data_training_str) == 161):

                    # check for classify or regression
                    if(classification_status):
                        write_tocsv(join_data_training_str, dataset_file_classification)
                        print("Classfication.. write records ", label_class)
                    else:
                        write_tocsv(join_data_training_str, dataset_file_regression)
                        print("Regression.. write records ", label_regression)

                else:
                    print("Packet not valid...")

                #print(join_data_training_str)
                print("Length : " + str(len(join_data_training_str)))
                count_incoming = 0  # reset count incoming
                join_data_training_str = np.array([] , dtype=str)


        # outer
        count_incoming = count_incoming + 1 # count for 2 incoming data

def define_class(topic):
    # normal walking
    if (topic == 'walking'):
        return '0'
    # stand
    elif (topic == 'stand'):
        return '1'
    # from stand to sit
    elif (topic == 'sit'):
        return '2'
    # from stand to terlentang
    elif (topic == 'terlentang'):
        return '3'

def strip_list_noempty(mylist):
    newlist = (item.strip() if hasattr(item, 'strip') else item for item in mylist)
    return [item for item in newlist if item or not hasattr(item, 'strip')]

def convert_data(raw_data):
    # testing features
    features = np.array([], dtype=float)
    # convert to float
    for x in range(len(raw_data) - 1):
        # print(float(signal[x]))
        features = np.hstack((features, float(raw_data[x])))

    return features

"""
Testing Signal
"""
def testing_signal(path_dir , signal):
    """
    Load model and test incoming signal
    :param path_dir: directory of model
    :param signal: list of signal
    :return: Result of testing
    """
    # check model path

    if os.path.isfile(path_dir):
        model = joblib.load(path_dir)
        #data_test = np.array([], dtype=float)
        #data_test = np.vstack((signal,signal))
        #data_test = np.vstack((data_test,signal))
        #print(data_test)
        predicted = model.predict(signal.reshape(1,-1))
        #pred_prob = model.predict_proba(signal.reshape(1,-1))
        #print(pred_prob)
        return predicted
    else:
        print("Model not found..., Train new one...")


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

    # topic for callibration
    print "Subscribe topic ", mqtt_topic_callibration
    clientMQTT.subscribe(mqtt_topic_callibration)
    print "Subscribe topic ", mqtt_topic_callibration_acc
    clientMQTT.subscribe(mqtt_topic_callibration_acc)
    print "Subscribe topic ", mqtt_topic_callibration_gyro
    clientMQTT.subscribe(mqtt_topic_callibration_gyro)


def main():
    # mqtt instance
    print("Server starting...")
    print("IP Broker : " + broker_host + ":" + str(broker_port))
    # create client
    global clientMQTT
    clientMQTT = mqtt.Client("server_fall_3")

    # set callback
    clientMQTT.on_message = on_message
    clientMQTT.on_connect = on_connect

    # connection established
    clientMQTT.connect(broker_host, broker_port)  # connect to broker

    clientMQTT.loop_forever()  # loop forever

if __name__ == "__main__":
    main()
