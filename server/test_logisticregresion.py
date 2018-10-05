from sklearn import metrics , datasets
from sklearn.linear_model import LogisticRegression

logisticRegr = LogisticRegression()

dataset = datasets.load_iris()

logisticRegr.fit(dataset.data , dataset.target)

y_real = dataset.target
y_predict = logisticRegr.predict(dataset.data)

print(metrics.classification_report(y_real, y_predict))
print(metrics.confusion_matrix(y_real, y_predict))

