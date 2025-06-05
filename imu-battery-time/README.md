## BLE Data Example

Service Description
| **BLE Service Description**                | **UUID**                             | Size     | BLE Byte ordder                                                                                                                                                                                        | Data Example                                                                                                           |
| ------------------------------------------ | ------------------------------------ | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------- |
| DESCRIPTOR_UUID                            | 2901                                 | \-       | \-                                                                                                                                                                                                     | \-                                                                                                                     |
| Board Version Information                  | 14A168D7-04D1-6C4F-7E53-F2E807B11900 | 32 bytes | [0] : Major<br>[1] : Minor<br>[2] : Patch<br>[3-9] : Sha-1<br>                                                                                                                                         | 02-00-00-39-34-38-65-63-36-38-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00<br>(2.0.0-948ec68)<br> |
| PC Time Sync                               | 14A168D7-04D1-6C4F-7E53-F2E802B11900 | 4 bytes  | Time Stamp<br>(Little-Endian)<br><br>                                                                                                                                                                  | 78-56-34-12                                                                                                            |
| IMU & Micro Data                           | 14A168D7-04D1-6C4F-7E53-F2E801B11900 | 38 bytes | [0-7] TimeStamp<br>[8-9] Device ID<br>[10-13] accelX<br>[14-17] accelY<br>[18-21] accelZ<br>[22-25] gyroX<br>[26-29] gyroY<br>[30-33] gyroX<br>[34-35] Microphone Level<br>[36-37] Microphone Peak<br> | See example                                                                                                            |
| Battery Level                              | 2A19                                 | 1 byte   | Battery Level<br>[0-100]<br>                                                                                                                                                                           | 5B (91%)                                                                                                               |
| Battery Charging Status                    | 14A168D7-04D1-6C4F-7E53-F2E803B11900 | bool     | True: Charging<br>False : Not Charging<br>                                                                                                                                                             | True                                                                                                                   |
| Microphone Control<br>(Default Enable)<br> | 14A168D7-04D1-6C4F-7E53-F2E806B11900 | bool     | True: Enable Mic<br>False : Disable Mic<br>                                                                                                                                                            | True                                                                                                                   |
| Get Device Maximum Loop Time               | 14A168D7-04D1-6C4F-7E53-F2E808B11900 | uint32   | Timestamp (us)                                                                                                                                                                                         | 00-00-44-EE<br>(17.646 ms)<br>                                                                                         |
| Reset Device Maximum Loop Time             | 14A168D7-04D1-6C4F-7E53-F2E809B11900 | bool     | True : Reset Loop Time                                                                                                                                                                                 | true                                                                                                                   |

MU & MIC Data every 100ms

When sent over BLE, the data is transmitted as a binary payload rather than text. Here's what the binary data structure looks like:

```

D9 05 16 41 96 01 00 00 91 44 3D DE 0E 06 BF 7B E8 31 3E 34 E1 48 41 47 18 C3 43 11 48 6A C2 0E FB 3C 0a 00 2a 00
```

Each Message contains 10 values  (38 bytes)

- `D9 05 16 41 96 01 00 00`: timestamp = 2025/04/17 00:11:24.505 UTC
- `91 44`: Device ID : 44 91 (Get from Chip)
- `3F 45 81 0A`: accelX = 0.772g (forward acceleration)
- `BD 8C A3 B6`: accelY = -0.069g (slight sideways)
- `40 0C 28 F6`: accelZ = 2.191g (upward acceleration)
- `42 3E 19 9A`: gyroX = 47.525 dps (rotation around X)
- `41 F2 28 F6`: gyroY = 30.270 dps (rotation around Y)
- `C2 12 E1 48`: gyroZ = -36.720 dps (rotation around Z)
- `0A 00`: Microphone Level : 10
- `2A 00`: Microphone Peak : 42
