#!/usr/bin/python3

import tensorflow as tf
import os, random, keras, numpy as np

f = open('/tmp/pm-mldata', 'rb')
raw = np.fromfile(f, dtype=np.float32).reshape((-1, 10))

f.close()

data = raw[:, :6] / np.array([50, 50, 20, 20, 1, 1])

fmean = np.mean(data, axis=0)
fstd = np.max(data, axis=0) - np.min(data, axis=0)
data = (data - fmean) / fstd

print(fmean)
print(fstd)

length = data.shape[0]
episode = 2000

train = []
valid = []

random.seed(10)

for index in range(0, length, episode):
    if index + episode < length:
        if random.uniform(0, 100) < 70:
            train.append(data[index:index + episode, :])
        else:
            valid.append(data[index:index + episode, :])

train_data = np.stack(train)
valid_data = np.stack(valid)

X = train_data[:, :, :4]
Y = train_data[:, :, 4:6]
vX = valid_data[:, :, :4]
vY = valid_data[:, :, 4:6]

print(X.shape)
print(Y.shape)
print(vX.shape)
print(vY.shape)

inputs = keras.layers.Input(shape=(None, 4))
gru_out = keras.layers.GRU(20, return_sequences=True) (inputs)
sincos = keras.layers.Dense(2, activation='tanh') (gru_out)
outputs = [sincos]
model = keras.models.Model(inputs, outputs)

model.compile(optimizer=keras.optimizers.Adam(),
        loss=keras.losses.MeanSquaredError())

model.summary()

model.fit(X, Y, validation_data=(vX, vY), epochs=300, batch_size=8)

#callbacks=keras.callbacks.EarlyStopping(patience=10)
