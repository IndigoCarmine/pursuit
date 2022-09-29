from statistics import mode
from typing import ParamSpecArgs
from keras.models import Sequential
from keras.layers import Convolution2D, MaxPooling2D
from keras.layers import Activation, Dropout, Flatten, Dense

import rospy
class Sleep:
    def __init__(self) -> None:
        self.model=None
        self.generate_model()
    
    def file_load():
        ParamSpecArgs

    def generate_model(self):
        model = Sequential()
        model.add(Convolution2D(32, 3, 3, input_shape=(128, 128, 3)))
        model.add(Activation('leaky_relu'))
        model.add(MaxPooling2D(pool_size=(2, 2)))

        model.add(Convolution2D(64, 3, 3))
        model.add(Activation('leaky_relu'))
        model.add(MaxPooling2D(pool_size=(2, 2)))

        model.add(Flatten())
        model.add(Dense(64))
        model.add(Activation('leaky_relu'))
        model.add(Dropout(0.5))
        model.add(Dense(2))
        model.add(Activation('softmax'))
        model.summary()

        model.compile(loss='categorical_crossentropy',
               optimizer='adam',
               metrics=['accuracy'])
        self.model = model
        
        




def main():
    rospy.init_node('sleep_node', anonymous=True)
    ln = Sleep()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown now")

if __name__ == '__main__':
    main()
