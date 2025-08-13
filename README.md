# signal-to-can_device
Measures analog and digital signals and converts readings to CAN bus messages. Easilly powered by 12V automotive systems to make CAN bus datalogging in your vehicle easy.

Power: 12V ~20 mA draw typical.

# CAN Messages

All messages are big-endian (Motorola).

## Output Messages

| Name             | ID            | DLC     | Bytes 0-1 | Bytes 2-3 | Bytes 4-5 | Bytes 6-7 |
| -----------      | -----------   | ------- | --------- | --------- | --------- | --------- |
| ADC Values 0 - 3 | node_id + 0x1 | 8 bytes | adc_0     | adc_1     | adc_2     | adc_3     |
| ADC Values 4 - 7 | node_id + 0x2 | 8 bytes | adc_4     | adc_5     | adc_6     | adc_7     |

| Name          | ID            | DLC     | Bytes 0-1  | Bytes 2-3 | Bytes 4-5 | Bytes 6-7  |
| -----------   | -----------   | ------- | ---------- | --------- | --------- | ---------- |
| Device Status | node_id + 0x3 | 8 bytes | adc_status | uptime    | v_supply  | fw_version |

### Parameters

**adc_0 to adc_7:** uint16, ontains the output value for each analog channel.

**adc_status:** uint16, Bit structure containing the current status for each channel. Each channel gets two bits starting with channel 0 to channel 7.

| 2-bit value  | Status            |
| -----------  | ----------------- |
| 00           | Inactive          |
| 01           | Active            |
| 10           | Out-of-range low  |
| 11           | Out-of-range high |

Example: adc_satus = 00011001010111 -> channel 0 inactive, channel 1 active, channel 3 out-of-range low, channel 4 active, channel 5 active, channel 6 active, channel 7 out-of-range high.

**uptime:** uint16, Seconds since power-on, wraps at 65535.

**v_supply:** uint16, Device supply voltage in mV.

**fw_version** uint16, Device firmware version.

## Command Message

A single CAN ID is used for all command messages. The device will respond with an acknoledge message after receiving the command message.

| Name            | ID            | DLC     | Byte 0  | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| -----------     | ------------- | ------- | ------- | -----  | -----  | -----  | -----  | -----  | -----  | -----  |
| Command Message | node_id + 0x5 | 8 bytes | com     | d1     | d2     | d3     | d4     | d5     | d6     | d7     |
| Ack Message     | node_id + 0x6 | 1 bytes | success |        |        |        |        |        |        |        |

### Command Codes

| Command Name       | Command Code (com) | d1             | d2             | d3              | d4              | d5         | d6        | d7 |
| ------------------ | ------------------ | -------------- | -------------- | --------------- | --------------- | ---------- | --------- | -- |
| SET_BAUD           | 1                  | baud_enum      | 0              | 0               | 0               | 0          | 0         | 0  |
| SET_SAMPLE_RATE    | 2                  | sample_rate[0] | sample_rate[1] | 0               | 0               | 0          | 0         | 0  |
| SET_CHANNEL        | 3                  | channel        | enable_bit     | scale_factor[0] | scale_factor[1] | offset[0]  | offset[1] | 0  |
| SET_CHANNEL_RANGE  | 4                  | channel        | oor_min[0]     | oor_min[1]      | oor_max[0]      | oor_max[1] | 0         | 0  |
| GET_VALUE          | 5                  | channel        | selection      | 0               | 0               | 0          | 0         | 0  |

### Parameters

**com:** uint8, Command code. Use the table below to specify a particular command and structure the data

**d1 to d5 :** unint8, Command-specific data.

**success:** uint8, 1 if command executed successfully, 0 if there was an error.

**baud_enum:** uint8, Enum representing the desired baud rate.

| baud_enum | Baud Rate |
| --------- | --------- |
| 0         | 125 kbps  |
| 1         | 250 kbps  |
| 2         | 500 kbps  |
| 3         | 1 Mbps    |

**sample_rate:** uint16, Sample rate of the device in Hz. All channels are sampled at the same rate.

**channel:** uint8, Channel number from 0 to 7.

**enable_bit:** uint8, 0 -> inactive, 1 -> active.

**scale_factor:** uint16, Scale factor to convert the channel voltage to a useful value.

**offset:** uint16, Channel offset value, used with scale factor to convert the channel voltage to a useful value.

**oor_min:** uint16, Out of range minimum value. Used to detect issues with the channel, wiring, or sensor. Specify as a voltage in milivolts, from 0 to 3300. Must be less than the oor_max for that channel.

**oor_max:** uint16, Out of range maximum value. Used to detect issues with the channel, wiring, or sensor. Specify as a voltage in milivolts, from 0 to 3300.

**selection:** uint8, Enum representing the desired value to be returned.

| selection | Value to be returned |
| --------- | -------------------- |
| 0         | sample_rate          |
| 1         | scale_factor         |
| 2         | offset               |
| 3         | oor_min              |
| 4         | oor_max              |