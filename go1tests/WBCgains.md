# WBC gains
Bold gains mean resulted ok. Italic gains means under test.

## Testing var: x
#### Test: multiple sinusoid

|   | x     | y | z | R | P | Y |
|---|---    |---|---|---|---|---|
| Kp|*600*  |100|100| 20| 20| 20|
| Kd| *25*  |  6|  6|0.5|0.5|0.5|
Result: ok


## Testing var: z 
####Test: half sinusoid

|   | x     | y | z   | R | P | Y |
|---|---    |---|---  |---|---|---|
| Kp|600|100|*700*| 20| 20| 20|
| Kd| 25|  6| *25*|0.5|0.5|0.5|
Result: not tracked. Increase Kp

---
####Test: half sinusoid
|   | x     | y | z   | R | P | Y |
|---|---    |---|---  |---|---|---|
| Kp|600|100|*900*| 20| 20| 20|
| Kd| 25|  6| *25*|0.5|0.5|0.5|
Result: slow tracking. Increase Kd

---
####Test: half sinusoid
|   | x     | y | z   | R | P | Y |
|---|---    |---|---  |---|---|---|
| Kp|600|100|*900*| 20| 20| 20|
| Kd| 25|  6| *40*|0.5|0.5|0.5|
Result: better but still slow tracking. Increase Kp on z. Need to increase x and y too

---
####Test: half sinusoid
|   | x     | y   |  z   | R | P | Y |
|---|---    |---  |---   |---|---|---|
| Kp|*700*  |*300*|*1000*| 20| 20| 20|
| Kd| 25|  *20*  |  *40*|0.5|0.5|0.5|
Result: better but slow tracking. Increase Kd

---
####Test: half sinusoid
|   | x     | y   |  z   | R | P | Y |
|---|---    |---  |---   |---|---|---|
| Kp|700|300|1000| 20| 20| 20|
| Kd| 25|  20  |  *60*|0.5|0.5|0.5|
Result: better but slow tracking. Increase Kd

---
####Test: half sinusoid
|   | x     | y   |  z   | R | P | Y |
|---|---    |---  |---   |---|---|---|
| Kp|700|300|1000| 20| 20| 20|
| Kd| 25|  20  |  *90*|0.5|0.5|0.5|
Result: better but slow tracking. Increase Kd

---
####Test: half sinusoid
|   | x     | y   |  z   | R | P | Y |
|---|---    |---  |---   |---|---|---|
| Kp|800|300|1000| 20| 20| 20|
| Kd| 25| 20|   *120*|0.5|0.5|0.5|
Result: slow tracking. Increase Kp, Kd

---
####Test: half sinusoid
|   | x     | y   |  z   | R | P | Y |
|---|---    |---  |---   |---|---|---|
| Kp|800|300|*1200*| 20| 20| 20|
| Kd| 25| 20| *120*|0.5|0.5|0.5|
Result: better but angles get wrong. Increase Kp, Kd for angles

---
####Test: half sinusoid
|   | x     | y   |  z   | R | P | Y |
|---|---    |---  |---   |---|---|---|
| Kp|800|300|1200| *50*| *50*| *50*|
| Kd| 25| 20| 120|  *2*|  *2*|  *2*|
Result: really better but still so slow. Increase Kd for z and Kp for x

---
####Test: half sinusoid
|   | x     | y   |  z   | R | P | Y |
|---|---    |---  |---   |---|---|---|
| Kp|900|300|1200|50|50|50|
| Kd| 25| 20|   *200*|  2|  2|  2|
Result: vibrations. Kd for z is too high. Reduce Kd and increase Kp

---
####Test: half sinusoid
|   | x     | y   |  z   | R | P | Y |
|---|---    |---  |---   |---|---|---|
| Kp|900|300|*1400*|50|50|50|
| Kd| 25| 20| *100*|  2|  2|  2|
Result: Increase Kp

---
####Test: half sinusoid
|   | x     | y   |  z   | R | P | Y |
|---|---    |---  |---   |---|---|---|
| Kp|900|300|*1600*|50|50|50|
| Kd| 25| 20| 100|  2|  2|  2|
Result: Still having large oscillations on x. Increasing its gains. Increasing sin ampl to 5 cm

---
####Test: half sinusoid
|   | x     | y   |  z   | R | P | Y |
|---|---    |---  |---   |---|---|---|
| Kp|1000|300|1600|50|50|50|
| Kd| 40| 20| 100|  2|  2|  2|
Result: really better but not satisfactory. too much friction. try to increase gains for y, to be sym