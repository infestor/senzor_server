# senzor_server
Linux app to be run in background as my wireless sensor net interface

Accepts incoming socket connections and serves as middle-point between the sensor network and anyone, who wants some data from sensors.
Receives requests as text commands and returns result (temperature, state of door switch etc.)
Periodically obtains data from sensors, so when you need it, they are always fresh
