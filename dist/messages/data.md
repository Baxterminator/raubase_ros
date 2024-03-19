# Raubase Sensor Data Messages

This page registers the custom sensors data messages.

## raubase_msgs::msg::DataDistance

This messages contains the raw value for one IR sensor.

=== "IDL"

    ```
    # This message contains the data of one of the plugged sensor
    builtin_interfaces/Time stamp

    char UNKNOWN = 0
    char SHARP = 1
    char URM09 = 2

    char type 1

    float64[2] calib
    float64 range
    float64 range_ad
    ```

=== "C++"

    ```c++
    struct DataDistance {
      static constexpr const char UNKNOWN{0};
      static constexpr const char SHARP{1};
      static constexpr const char URM09{2};

      builtin_interfaces::msg::Time stamp;
      char type = SHARP;
      double calib[2];
      double range;
      double range_ad; 
    };
    ```

=== "Python"

    ```Python
    class DataDistance:
        UNKNOWN = 0
        SHARP = 1
        URM09 = 2

        def __init__(self):
            self.stamp : builtin_interfaces.msg.Time
            self.type : int
            self.calib : Tuple[float, float]
            self.range : float
            self.range_ad: float
    ```

## raubase_msgs::msg::DataEncoder

This messages contains both encoders position.

=== "IDL"

    ```
    builtin_interfaces/Time stamp

    int32 right
    int32 left
    ```

=== "C++"

    ```c++
    struct DataEncoder {
      builtin_interfaces::msg::Time stamp;
      int right;
      int left; 
    };
    ```

=== "Python"

    ```Python
    class DataEncoder:
        def __init__(self):
            self.stamp : builtin_interfaces.msg.Time
            self.right : int
            self.left  : int
    ```

## raubase_msgs::msg::DataLineSensor

This messages the raw reading from the line sensor.

=== "IDL"

    ```
    builtin_interfaces/Time stamp

    # Values of each sensor of the sensor line
    int32[] data
    ```

=== "C++"

    ```c++
    struct DataLineSensor {
      builtin_interfaces::msg::Time stamp;
      std::vector<int> data;
    };
    ```

=== "Python"

    ```Python
    class DataLineSensor:
        def __init__(self):
            self.stamp : builtin_interfaces.msg.Time
            self.data : List[int]
    ```