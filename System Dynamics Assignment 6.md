## System Dynamics Assignment 6

Ruxiao Chen, Guanlin Wu

### Problem 1

a. ![P_1](/Users/wuguanlin/Desktop/JHU/2025 Spring/System Dynamics/Assignment/ASS_6/P_1.png)

**Staff:**

- Equation: `Training Completion Rate - Leaving Rate`
- Units: People
- Initial Value: 30

**Staff in Training:**

- Equation: `Recruitment Rate - Training Completion Rate`
- Units: People
- Initial Value: 0

**Recruitment Rate:**

- Equation: `Inertial Control+DELAY1(Staff Shortage/Recruitment Delay, Recruitment Delay)`
- Units: People/Month
- This equation incorporates the inertial control element and a first-order delay of the "Staff Shortage" divided by the "Recruitment Delay". The `DELAY1` function represents a first-order information delay. The third argument ensures that the initial output of the delay is the initial value of the input.

**Training Completion Rate:**

- Equation: `DELAY3(Staff in Training, Training Delay)`
- Units: People/Month
- This implements a third-order material delay. Individuals enter "Staff in Training" and exit to "Staff" after a 4-month average delay, with a more gradual outflow pattern than a first-order delay.

**Leaving Rate:**

- Equation: `Staff * Average Leaving Rate`
- Units: People/Month

**Target Staff:**

- Units: People
- 150

**Average Leaving Rate:**

- 0.05
- Units: 1/Month
- `Constant Leaving Fraction`: 0.05 * staff

**Training Delay:**

- Equation: `4`
- Units: Month

**Recruitment Delay:**

- Equation: `5`
- Units: Month

**Staff Shortage:**

- Equation: `Target Staff - Staff`
- Units: People

**Proportional Recruitment:**

- Equation: `Staff Shortage / Recruitment Delay`
- Units: People/Month

**Inertial Control:**

- Equation: `SMOOTH(leaving rate, 3)`
- Units: People/Month
- `Inertia Time`: 3

b. Inertial control

c. ![P_1_c](/Users/wuguanlin/Desktop/JHU/2025 Spring/System Dynamics/Assignment/ASS_6/P_1_c.png)

d. 