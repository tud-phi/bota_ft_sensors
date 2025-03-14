# bota sensone force-torque sensor

this directory contains useful information about using the [https://www.botasys.com/force-torque-sensors/sensone](bota sensone) force-torque sensor.

### usage

the main script is `sensone.py` which is a cleaner version of the `bota_serial_example.py` script [https://gitlab.com/botasys/python_interface/-/blob/main/examples/bota_serial_example.py](provided by bota).

there is a tiny example of usage in the main script that looks like this:

```python
port = "/dev/ttyUSB0"
sensor = BotaSerialSensor(port)
sensor.start()

try:
    while True:
        data = sensor.reading
        print(data)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("stopped")

sensor.close()
```

select the correct port for the sensor and run the script. for checking the usb port of the sensor, you can use:

```bash
ls /dev/ttyUSB*
```

you need to call the `sensor.start()` method to start the reading of the sensor on a thread, and then you can access the data by calling the `sensor.reading` attribute. it is also important to close the sensor connection by calling the `sensor.close()` method.

the reading is a `NamedTuple` (fancy class) that has the following attributes:

```python
class Reading(NamedTuple):
    fx: float
    fy: float
    fz: float
    mx: float
    my: float
    mz: float
    timestamp: float
    temperature: float
```

access the data by calling the attribute of the reading, for example, `data.fx` will give you the force in the x-axis.


### configuring the sensor

in the script there is also a class that contains the configuration of the sensor, the `SensorConfig` class. it has the following attributes:

```python
class Config(NamedTuple):
    baud_rate: int
    sinc_length: int
    chop_enable: int
    fast_enable: int
    fir_disable: int
    temp_compensation: int  # 0: Disabled (recommended), 1: Enabled
    use_calibration: int  # 1: calibration matrix active, 0: raw measurements
    data_format: int  # 0: binary, 1: CSV
    baud_rate_config: int  # 0: 9600, 1: 57600, 2: 115200, 3: 230400, 4: 460800
    frame_header: bytes
```

bota recommends to use the default configuration, which is loaded by default, so you don't need to change anything.

i haven't been able to make other configurations work, so if you want to try, let us know if you succeed.

### calibration

the raw readings of the sensor are a bit noisy and don't start at zero. i recommend the following calibration procedure:

1. pass the reading through a filter (low-pass, kalman, median, exponential, etc.) and store the filtered readings in a list. one second of readings is enough.

2. discard the first readings of the list to be safe that the sensor is stable.

3. calculate the mean of the readings. that is you initial offset.

4. subtract the offset from the readings to make them relative.

`kalman_filter.py` is a simple kalman filter that you can use to filter the readings. i recommend using the `MultiKalmanFilter` class, which is a kalman filter that can handle multiple readings at the same time.

`median_filter.py` provides also a `MultiMedianFilter` class that can handle multiple readings at the same time. i have not tested this class, but it should work.


### ros wrapper

`senseone_ros2.py` WIP

### troubleshooting

if you are having trouble connecting to the sensor, try the following:

1. make sure the sensor is connected to the computer via USB (ask tomás)

2. `chmod +x /dev/ttyUSB0` to give permissions to the USB port

3. `chmod +x sensone.py` to give permissions to the python script

if you have more problems, please, submit an issue in this repository.


### other

- remember that the sensor is mounted rotated 45 degrees along the z-axis of the sensor (at least on the panda robot). don't forget to undo this rotation in your calculations.
