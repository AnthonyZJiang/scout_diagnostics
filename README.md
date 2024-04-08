# scout_diagnostics
Converts `\scout_status` and `\BMS_status` topics published by [scout_ros](https://github.com/agilexrobotics/scout_ros) into `/diagnostics`, for faster integration with Foxglove Studio.

## TODO
Battery SOC based on voltage as Scout Mini does not seem to publish valid BMS_status.