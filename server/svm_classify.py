from sklearn import datasets, metrics , model_selection
from sklearn.naive_bayes import GaussianNB
from sklearn import svm
import cPickle as pickle
from sklearn.externals import joblib
import os
import csv
import gc
import time
import numpy as np
from my_db import *

# change
model_name = 'svm_classify.joblib.pkl'
path_dataset = 'dataset/fall_dataset_gabungan.csv'

def train_model(features, label):
    model_path = 'model/'
   # model_name = 'naive_bayes_6.joblib.pkl' # save name for model
    full_model_name = model_path + model_name
    if os.path.isfile(full_model_name):
        print("Already, training...\n Loading " + model_name + "...")
        model = joblib.load(full_model_name)
    else:
        start = time.time()
        model= svm.SVC()
        model.fit(features , label)
        end = time.time()
        require_time = end-start
        print("Trained completed! \n\t " + full_model_name + "\n" + "\t Time required : " + str(require_time) + " sec")

        #save jobpkl
        joblib.dump(model, full_model_name)

    return model

def load_dataset(path_dataset):
    file_dir = path_dataset.split('/')
    my_db_pickle_name = 'dataset/pickle/' + file_dir[1] + '.p'
    my_db = fall_db()  # instance mydb

    # check if already have process the signal before for fasting process..
    if os.path.isfile(my_db_pickle_name):
        print("Already have pickle.. \n" + my_db_pickle_name )
        f = open(my_db_pickle_name, 'rb')
        # disable garbage collector
        gc.disable()  # this improve the required loading time!
        my_db = pickle.load(f)
        gc.enable()
        f.close()
    else:
        print("Load signal... \n " + path_dataset)
        f = open(path_dataset,'rU')
        reader = csv.reader(f, delimiter=',')
        data = []
        label = []
        for row in reader:
            print("read row..")
            data_float = convert_data(row[0:160])
            data.append(data_float)
            label.append(int(row[160]))

        f.close()

        # set data and label
        my_db.filename = file_dir[1]
        my_db.data = data
        my_db.label_class = label

        print("Saving signal processed data ...")
        f = open(my_db_pickle_name, 'wb')
        pickle.dump(my_db, f, 2) # save my db as pickle
        f.close

    return my_db

def convert_data(raw_data):
    # testing features
    features = np.array([], dtype=float)
    # convert to float
    for x in range(len(raw_data)):
        # print(float(signal[x]))
        features = np.hstack((features, float(raw_data[x])))

    return features

def main():
    #path_dataset = 'dataset/fall_dataset_gabungan.csv' # change with dataset
    data_train = load_dataset(path_dataset)
    svm_classify = train_model(data_train.data , data_train.label_class)

    predicted = svm_classify.predict(data_train.data)
    print()
    print(metrics.classification_report(data_train.label_class , predicted))
    print(metrics.confusion_matrix(data_train.label_class , predicted))

    print("Evaluation...")

    seed = 7
    kfold = model_selection.KFold(n_splits=3, random_state=seed)

    results = model_selection.cross_val_score(svm_classify, data_train.data, data_train.label_class, cv=kfold)
    print("Accuracy : " + str(results.mean()))

    """
    model = GaussianNB()
    model.fit(dataset.data, dataset.target)

    expected = dataset.target
    predicted = model.predict(dataset.data)

    print(metrics.classification_report(expected, predicted))
    print(metrics.confusion_matrix(expected, predicted))
    """
if __name__ == "__main__":
    main()