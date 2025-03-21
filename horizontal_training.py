import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.utils import to_categorical
import os
import pickle
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import classification_report, confusion_matrix

CSV_FILENAME = "horizontal_data.csv"
TFLITE_MODEL_FILENAME = "horizontal_model.tflite"
EPOCHS = 50
BATCH_SIZE = 16
TEST_SPLIT = 0.2
RANDOM_STATE = 42

if not os.path.isfile(CSV_FILENAME):
    raise FileNotFoundError(f"{CSV_FILENAME} not found.")
df = pd.read_csv(CSV_FILENAME)
print("Horizontal Data Preview:")
print(df.head())

X = df["value"].values.astype(np.float32)
X = X.reshape(-1, 1)
labels = df["label"].values
label_encoder = LabelEncoder()
y_int = label_encoder.fit_transform(labels)
y = to_categorical(y_int)
print("Unique labels:", label_encoder.classes_)

X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=TEST_SPLIT, random_state=RANDOM_STATE)

input_dim = 1
num_classes = y.shape[1]
model = Sequential([
    Dense(16, activation='relu', input_shape=(input_dim,)),
    Dense(16, activation='relu'),
    Dense(num_classes, activation='softmax')
])
model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
model.summary()

history = model.fit(X_train, y_train,
                    validation_split=0.1,
                    epochs=EPOCHS,
                    batch_size=BATCH_SIZE,
                    verbose=1)

loss, accuracy = model.evaluate(X_test, y_test, verbose=0)
print(f"Horizontal Model Test Accuracy: {accuracy*100:.2f}%")
y_pred = np.argmax(model.predict(X_test), axis=1)
y_true = np.argmax(y_test, axis=1)
print("Classification Report:")
print(classification_report(y_true, y_pred, target_names=label_encoder.classes_))
cm = confusion_matrix(y_true, y_pred)
plt.figure(figsize=(8,6))
sns.heatmap(cm, annot=True, fmt="d", cmap="Blues",
            xticklabels=label_encoder.classes_,
            yticklabels=label_encoder.classes_)
plt.xlabel("Predicted Label")
plt.ylabel("True Label")
plt.title("Horizontal Model Confusion Matrix")
plt.show()

plt.figure(figsize=(12,4))
plt.subplot(1,2,1)
plt.plot(history.history['loss'], label='Train Loss')
plt.plot(history.history['val_loss'], label='Val Loss')
plt.xlabel("Epoch")
plt.ylabel("Loss")
plt.title("Horizontal Model Loss")
plt.legend()
plt.subplot(1,2,2)
plt.plot(history.history['accuracy'], label='Train Accuracy')
plt.plot(history.history['val_accuracy'], label='Val Accuracy')
plt.xlabel("Epoch")
plt.ylabel("Accuracy")
plt.title("Horizontal Model Accuracy")
plt.legend()
plt.show()

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
tflite_model = converter.convert()
with open(TFLITE_MODEL_FILENAME, "wb") as f:
    f.write(tflite_model)
print(f"TFLite model saved to {TFLITE_MODEL_FILENAME}")

with open("horizontal_label_encoder.pkl", "wb") as f:
    pickle.dump(label_encoder, f)
print("Horizontal label encoder mapping saved to horizontal_label_encoder.pkl")
