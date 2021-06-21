Machine Learning:

- Consists of 

    1. a Linear Regression model which predicts the price of a car given its features

    2. a Logistic Regression model which predicts heart diseases

    3. Neural network made from scratch (without Keras+Tensorflow) implemented on the CIFAR-10 dataset



Deep Learning:

 - Consists of

    1. a Pretrained ResNet50 model which has been implemented on the EuroSAT dataset for distinguishing various landforms.

    2. a ResNet50 model trained from scratch for the same.



OpenCV:

 - Consists of 
 
   1. a Lane detection Algorithm



Final Task folder consists of the following mini project: 

    (Implemented in ROS+OpenCV and also OpenCV in CPP using scripts)

    You want to watch a movie while your car drives by itself. But the car should not let you get so lost in the movie that you crash.

    Implement a priority switching method to switch the display between the movie stream and the car’s camera stream.
    

    Implement 3 modules.

    Module 1: Responsible for publishing the movie stream.

    Module 2: Responsible for publishing the car’s camera stream.

    Module 3 (central): Responsible for detecting when a vehicle(car) is in our lane.
    

    If the vehicle occupies 45% x 45% of the pixels in our lane, then set priority to 3.

    If the vehicle occupies 15% x 15% of the pixels in our lane, then set priority to 2.

    If the vehicle occupies less than the above, then set priority to 1.
    

    If the priority value is:

    3: Only display the car’s camera stream

    2: Switch between the car’s camera stream and the movie stream, displaying each for 2 seconds

    1: Only display the movie stream.
