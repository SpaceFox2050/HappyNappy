## Inspiration

Naps that go on too long often cause grogginess


Grogginess leads to lower productivity after waking up


We wanted a way to wake up at the right time during a nap


## What it does

Sleep occurs in multiple stages during a nap


Entering REM sleep and being woken during it causes grogginess


The pillow detects when the user is approaching REM sleep


The user is woken before entering the grogginess-inducing stage


## How we built it

Used an ESP32 connected to a MAX30102 sensor


Collected heart rate and SpO₂ data


Sent raw infrared sensor data via MQTT to a backend processor


Used HeartPy to filter noise and discard invalid sensor values


Extracted a clean heartbeat dataset


Detected heart rate changes correlated with entering REM sleep


Sent a wake-up command back to the ESP32 once REM was detected


## Challenges we ran into

MQTT setup did not work on school Wi-Fi


Required using a personal router for testing


Unable to obtain REM sleep training data from sleepdata.org


Limited experience with backend development


Difficulty integrating hardware data with software processing

Lack of internet access when using personal router, leading to data having to be processed locally

## Accomplishments that we're proud of

Development of UI via an LED on the the pillow itself.
Wireless connection through a local network

## What we learned

Basics of MQTT connection setup
Better organization of online workspace as a team



## What's next for HappyNappy

Implement a more accurate REM detection model


Train the model using an online sleep data database


