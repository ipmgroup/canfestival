This utility was made to work with a controller running [this firmware](https://github.com/ipmgroup/c2000-motor).

To run the utility enter `./tctl` with the options you want. For example to show all available commands with a brief description, enter `./tctl -h`

## Profile configuration
### Default profiles
The controller has 5 pre-configured profiles for 4 different motors. The default profiles are the following:
| Profile | Motor type |
|---|---|
| 0 | ILM70_10 |
| 1 | T200_R1 |
| 2 | T100_R1 |
| 3 | lmt_1920_18 |
| 4 | ILM70_10 |

Procedure for switching profiles:
1. Set the controlword (0x6040) to 1: `./tctl -q1`
2. Send the desired profile number to the profile selection register (0x2000): `./tctl --active_profile=1`
3. Restart the device.

After this procedure, the controller should load the new profile during startup.

Note: to view the currently active profile without making any changes:
`./tctl --active_profile`

### Manual configuration
Each parameter of the profile can be set manually. This is done via the profile registers (0x2001.X). Some of these parameters cannot be determined automatically, and have to be set manually before using the automatic configuration, or the automatic configuration might fail.

Here is a list of all profile parameters:
| Address (0x2001.[*]) | Parameter | Automatic detection possible? |
|---|---|---|
| 1  | device_id          | No   |
| 2  | motor_type         | No   |
| 3  | motor_numPolePairs | No   |
| 4  | motor_Rr           | Yes  |
| 5  | motor_Rs           | Yes  |
| 6  | motor_Ls_d         | Yes  |
| 7  | motor_Ls_q         | Yes  |
| 8  | motor_ratedFlux    | Yes  |
| 9  | !IdRated           | No   |
| A  | maxCurrent_resEst  | No   |
| B  | maxCurrent_indEst  | No   |
| C  | maxCurrent         | No   |
| D  | fluxEstFreq_Hz     | No   |
| E  | rpmLimit           | No   |
| F  | rpmpsLimit         | No   |
| 10 | kp                 | No   |
| 11 | ki                 | No   |

Procedure for changing parameters:
1. Set the controlword (0x6040) to 1: `./tctl -q1`
2. Set the corresponding parameter (0x2001.X) to the desired value (each as a 32 bit IEEE 754 floating point number*): `./tctl --maxCurrent=20.5`
At this point the parameters are set in the RAM, but not persistent memory. To save the parameters to a profile, continue with the following steps:
3. [Optional] Select the profile to which the parameters will be saved via the profile selection register (0x2000): `./tctl --active_profile=1`
4. Send 1 to the profile save register (0x1010.1): `./tctl --save_profile=1`. Note: it is always 1 in this step, no matter the profile you want to save it to.
`*` There are range restrictions on some parameters, most notably kp (0 - 25.5 in steps of 0.1) and ki (0 - 0.255 in steps of 0.001).

After this procedure the new values are saved in the profile selected in step 3, or if the step is omitted, in whatever profile the controller loaded at startup (default: 0).

Note: to view the parameters in the currently active profile without making any changes (for example max current):
`./tctl --maxCurrent`

### Automatic configuration
Because of slight differences between individual motors, it is possible to automatically fine-tune some of the parameters. If the parameters that cannot be determined automatically (see table in the section for manual configuration) are set correctly, you can proceed with the automatic configuration.

Procedure for auto-configuration:
1. Set the controlword (0x6040) to 0: `./tctl -q0`. This step is important to reset some parameters that might otherwise send the auto-configuration into an infinite loop.
2. Set the controlword (0x6040) to 0x8003: `./tctl -q8003`
3. Wait for the statusword (0x6041) to be set to 0x8001 (if it is 0x8003, the operation is still in progress): `./tctl -v --statusword`. Keep in mind, in this step the motor will turn on, and might spin in an uncontrolled fashion.
At this point the motor should be operational, but the parameters are set only in the RAM, not persistent memory. To save the parameters to a profile, continue with the following steps:
4. [Optional] Select the profile to which the parameters will be saved via the profile selection register (0x2000): `./tctl --active_profile=1`
5. Send 1 to the profile save register (0x1010.1): `./tctl --save_profile=1`. Note: it is always 1 in this step, no matter the profile you want to save it to.

After this procedure the new values are saved in the profile selected in step 4, or if the step is ommitted, in whatever profile the controller loaded at startup (default: 0).
