# [202220833-최우성] ICC 제어기 설계 보고서

**과목**: 자동제어 - 2026 봄  
**제출일**: 2026-06-01  
**팀**: 개인

---

## 1. 설계 개요

본 프로젝트의 목표는 통합 샤시 제어(Integrated Chassis Control, ICC)를 통해 차량의 횡방향 안정성, 제동 안정성, 차체 수직방향 안정성을 동시에 개선하는 것이다. 제공된 검증 환경은 14DOF 차량 모델을 포함하지만, 제어기 설계에서는 복잡한 전체 차량 모델을 직접 제어 대상으로 삼기보다 bicycle model, 점질량 종방향 모델, quarter-car 기반 수직방향 모델을 사용하여 단순화하였다. 이후 run_icc_benchmark.m 및 grade.m을 통해 14DOF 환경에서 제어 성능을 검증하였다.

본 설계에서는 네 개의 제어 파일을 역할별로 분리하였다. ctrl_lateral.m은 yaw rate tracking과 body slip angle 억제를 담당하고, ctrl_longitudinal.m은 속도 및 제동 안정성을 담당한다. ctrl_vertical.m은 skyhook 기반 semi-active damping을 통해 수직방향 진동을 줄이며, ctrl_coordinator.m은 세 제어기의 출력을 starter code의 구조와 동일하게 전달한다.

특히 채점기에서 요구하는 latCmd, lonCmd, verCmd의 구조를 유지하기 위해 coordinator에서 임의의 actuator allocation이나 plant 입력 재구성은 수행하지 않았다. 이는 실행 오류나 인터페이스 불일치에 의한 감점을 방지하기 위한 설계 선택이다.

선택한 제어기법은 다음과 같다.

- **ctrl_lateral**: yaw rate tracking PID + gain scheduling + beta-limiter 기반 ESC
- **ctrl_longitudinal**: 속도 오차 기반 PI 제어 + jerk limit + ABS-like modulation
- **ctrl_vertical**: skyhook logic 기반 semi-active CDC damping 제어
- **ctrl_coordinator**: latCmd, lonCmd, verCmd를 starter 구조와 동일하게 전달하는 pass-through coordinator

이러한 구조를 선택한 이유는 차량 안정성 문제에서 yaw rate, slip angle, 제동 slip, 수직방향 진동이 서로 다른 물리량이지만, 실제 시나리오에서는 동시에 영향을 주기 때문이다. 따라서 각 축별 제어기를 독립적으로 설계하고, coordinator에서는 출력 구조만 유지하여 인터페이스 오류와 자동 감점을 방지하였다.

---

## 2. 수학적 모델링

### 2.1 사용한 plant 단순화

실제 검증 plant는 14DOF 차량 모델이지만, 제어기 설계 단계에서는 횡방향 bicycle model을 사용하였다. Bicycle model은 차량의 lateral velocity와 yaw rate를 중심으로 차량의 횡방향 거동을 설명하는 단순 모델이다. 이 모델은 차량 안정성 제어에서 가장 중요한 yaw rate response와 side slip angle을 해석하기에 적합하다.

종방향 제어는 차량을 질량 m을 갖는 점질량으로 근사하였다. 목표 속도와 현재 속도의 차이로부터 요구 가속도를 계산하고, 이를 전체 종방향 힘 Fx로 변환하였다. 수직방향 제어는 각 바퀴의 sprung mass velocity와 unsprung mass velocity를 이용하는 quarter-car 기반 skyhook 개념을 사용하였다.

### 2.2 횡방향 bicycle model

횡방향 제어 설계를 위한 상태 변수는 다음과 같이 설정하였다.

```text
x = [vy, r]^T

vy : lateral velocity
r  : yaw rate
```

제어 입력은 전륜 조향각 delta와 yaw moment Mz로 보았다.

```text
u = [delta, Mz]^T

delta : additional front steering command
Mz    : requested yaw moment
```

선형 타이어 모델과 일정한 종방향 속도 Vx를 가정하면 횡방향 운동방정식은 다음과 같이 표현할 수 있다.

```text
d(vy)/dt = -(Cf + Cr)/(m*Vx)*vy
           + ((lr*Cr - lf*Cf)/(m*Vx) - Vx)*r
           + (Cf/m)*delta

d(r)/dt  = (lr*Cr - lf*Cf)/(Iz*Vx)*vy
           - (lf^2*Cf + lr^2*Cr)/(Iz*Vx)*r
           + (lf*Cf/Iz)*delta
           + (1/Iz)*Mz
```

이를 state-space 형태로 쓰면 다음과 같다.

```text
dx/dt = A*x + B*u

A11 = -(Cf + Cr)/(m*Vx)
A12 = (lr*Cr - lf*Cf)/(m*Vx) - Vx
A21 = (lr*Cr - lf*Cf)/(Iz*Vx)
A22 = -(lf^2*Cf + lr^2*Cr)/(Iz*Vx)

B11 = Cf/m
B12 = 0
B21 = lf*Cf/Iz
B22 = 1/Iz
```

출력은 yaw rate와 body slip angle을 중심으로 설정하였다.

```text
y = [r, beta]^T

small angle approximation:
beta ≈ vy / Vx
```

### 2.3 종방향 단순 모델

종방향 제어에서는 차량을 질량 m을 갖는 점질량으로 보았다.

```text
m * d(Vx)/dt = Fx
```

속도 오차는 다음과 같다.

```text
ev = Vx_ref - Vx
```

PI 제어기를 통해 요구 종방향 힘을 계산하였다.

```text
Fx_raw = m * (Kp * ev + Ki * integral(ev))
```

이때 실제 차량에서는 tire-road friction, brake actuator, wheel slip 등의 제한이 존재하므로, 본 설계에서는 최대 가속도 제한과 jerk limit을 추가하였다.

### 2.4 수직방향 quarter-car 기반 모델

수직방향 제어에서는 각 바퀴에 대해 sprung mass velocity zs_dot, unsprung mass velocity zu_dot을 사용하였다. Suspension relative velocity는 다음과 같다.

```text
relVel = zs_dot - zu_dot
```

Skyhook 제어에서는 차체가 가상의 고정된 기준점에 damper로 연결된 것처럼 작동하도록 감쇠계수를 조절한다. 본 설계에서는 다음 조건을 사용하였다.

```text
if zs_dot * relVel > 0:
    damping = cMax
else:
    damping = cMin
```

위 조건을 만족하면 damper가 차체 진동 에너지를 줄이는 방향으로 작용할 수 있으므로 큰 감쇠계수 cMax를 사용하고, 그렇지 않으면 작은 감쇠계수 cMin을 사용하였다.

### 2.5 가정 및 한계

본 설계에서 사용한 가정은 다음과 같다.

- 제어기 설계 단계에서는 종방향 속도 Vx가 짧은 시간 동안 일정하다고 가정하였다.
- 타이어는 작은 slip angle 영역에서 선형 cornering stiffness를 갖는다고 가정하였다.
- lateral, longitudinal, vertical dynamics는 설계 단계에서 분리하여 다루었다.
- 실제 검증 plant는 14DOF이므로 하중 이동, 타이어 비선형성, actuator saturation, 제동 중 횡방향-종방향 결합 효과가 존재한다.
- 따라서 단순 모델 기반 설계와 실제 시뮬레이션 결과 사이에는 차이가 발생할 수 있다.

이러한 한계를 보완하기 위해 모든 제어 명령에 saturation, rate limit, gain scheduling, anti-windup을 적용하였다.

---

## 3. 제어기 설계

### 3.1 ctrl_lateral - AFS + ESC

#### 설계 목표

ctrl_lateral의 설계 목표는 yaw rate tracking 성능과 차량 안정성 확보이다. 구체적으로는 yaw rate reference를 빠르게 추종하되, body slip angle이 커지는 경우 yaw moment를 이용해 차량의 회전 불안정성을 억제하는 것이다.

#### AFS yaw rate PID

Yaw rate error는 다음과 같이 정의하였다.

```text
er = r_ref - r
```

AFS 조향 보정 명령은 PID 제어기를 통해 계산하였다.

```text
delta_raw = Kp*er + Ki*integral(er) + Kd*d(er)/dt
```

고속에서는 같은 조향각 변화가 더 큰 횡가속과 yaw response를 만들 수 있으므로, 조향 명령에는 속도 기반 gain scheduling을 적용하였다.

```text
steerSchedule = 1 / max(1.0, Vx/18.0)
```

최종 조향 명령은 다음과 같이 제한하였다.

```text
delta_cmd = sat(steerSchedule * delta_raw,
                -0.11 * maxSteer,
                 0.11 * maxSteer)
```

#### ESC beta-limiter

Body slip angle이 기준값보다 커지면 ESC yaw moment가 개입하도록 하였다. 기준값은 다음과 같다.

```text
betaTh = 3 deg
betaHard = 7 deg
```

Slip angle이 커지는 경우 반대 방향 yaw moment를 인가하여 slip angle 증가를 억제한다.

```text
if abs(beta) > betaTh:
    Mz_beta = -Kbeta * sign(beta) * (abs(beta) - betaTh) * escSchedule
else:
    Mz_beta = 0
```

ESC gain scheduling은 다음과 같이 설정하였다.

```text
escSchedule = min(max(Vx/15.0, 0.5), 2.2)
```

Yaw rate error에 대한 보조 yaw moment도 추가하였다.

```text
Mz_yaw = Kr * er * escSchedule
```

최종 yaw moment는 slip angle 상태에 따라 다음과 같이 결정하였다.

```text
if abs(beta) > betaTh:
    Mz_raw = Mz_beta + 0.35 * Mz_yaw
else:
    Mz_raw = 0.55 * Mz_yaw

Mz_cmd = sat(Mz_raw, -maxMz, maxMz)
```

#### 최종 적용 파라미터

```matlab
Kp = getFieldSafe(CTRL.LAT, 'Kp', 1.0);
Ki = getFieldSafe(CTRL.LAT, 'Ki', 0.1);
Kd = getFieldSafe(CTRL.LAT, 'Kd', 0.05);

steerScale = 0.11;
maxMz = 900;
maxMzRate = 60000;
Kbeta = 18000;
Kr = 250;

betaTh = deg2rad(3.0);
betaHard = deg2rad(7.0);
```

maxMz = 900, Kbeta = 18000, Kr = 250, steerScale = 0.11은 자동채점 결과를 기준으로 선택한 최종 조합이다. 이 조합은 A7 brake-in-turn에서 side slip을 크게 줄이면서도 A1/D1에서 lateral deviation이 과도하게 악화되지 않도록 하는 절충값이다.

### 3.2 ctrl_longitudinal - 속도 제어 + ABS-like modulation

ctrl_longitudinal은 목표 속도와 현재 속도 사이의 오차를 이용하여 전체 종방향 힘 Fx_total을 생성한다.

속도 오차는 다음과 같다.

```text
ev = Vx_ref - Vx
```

PI 제어식은 다음과 같다.

```text
Fx_raw = m * (Kp * ev + Ki * integral(ev))
```

종방향 힘은 최대 가속도 제한을 사용하여 제한하였다.

```text
Fx_max = m * ax_max
Fx_cmd = sat(Fx_raw, -Fx_max, Fx_max)
```

공개 runner의 ctrl_longitudinal 입력에는 wheel slip ratio가 직접 포함되어 있지 않다. 따라서 실제 ABS처럼 slip ratio를 직접 feedback하지는 못하고, 감속도 ax가 너무 커지는 경우 제동력을 줄이는 ABS-like modulation을 적용하였다.

```text
if Fx_raw < 0 and ax < -0.85*ax_max:
    Fx_raw = 0.65 * Fx_raw
elseif Fx_raw < 0 and ax < -0.65*ax_max:
    Fx_raw = 0.80 * Fx_raw
```

제동력 변화가 급격해지는 것을 막기 위해 jerk limit도 적용하였다.

```text
dFmax = m * maxJerk * dt
Fx_cmd = sat(Fx_raw, prevForce - dFmax, prevForce + dFmax)
```

최종 적용한 주요 파라미터는 다음과 같다.

```matlab
Kp = getFieldSafe(CTRL.LON, 'Kp', 0.35);
Ki = getFieldSafe(CTRL.LON, 'Ki', 0.03);
intMax = getFieldSafe(CTRL.LON, 'intMax', 2000);
m = 1500;
maxAx = getFieldSafe(LIM, 'MAX_AX', 10.0);
maxJerk = getFieldSafe(LIM, 'MAX_JERK', 50.0);
```

출력 구조는 starter와 동일하게 유지하였다.

```matlab
lonCmd.Fx_total = FxCmd;
lonCmd.brakeRatio = sat(abs(FxCmd) / maxForce, 0, 1);
```

### 3.3 ctrl_vertical - Skyhook CDC

ctrl_vertical은 4개 바퀴의 damping command를 생성한다. 본 설계에서는 skyhook logic을 사용하였다.

Suspension relative velocity는 다음과 같다.

```text
relVel = zs_dot - zu_dot
```

Skyhook 조건은 다음과 같다.

```text
if zs_dot * relVel > 0:
    damping = cMax
else:
    damping = cMin
```

조건을 만족하면 큰 감쇠계수 cMax, 그렇지 않으면 작은 감쇠계수 cMin을 사용하였다. 또한 damping command가 급격하게 바뀌는 것을 방지하기 위해 rate limit을 적용하였다.

최종 적용한 주요 파라미터는 다음과 같다.

```matlab
cMin = getFieldSafe(CTRL.VER, 'cMin', 500);
cMax = getFieldSafe(CTRL.VER, 'cMax', 5000);
cNom = getFieldSafe(CTRL.VER, 'cNom', 0.5 * (cMin + cMax));
relVelTh = 0.20;
maxRate = 80000;
```

출력은 다음과 같이 4x1 vector 구조로 유지하였다.

```matlab
verCmd = dampingCmd(:);
```

### 3.4 ctrl_coordinator - 구조 유지

ctrl_coordinator는 새로운 제어 계산을 수행하지 않고, 각 제어기 출력을 starter와 같은 구조로 전달하도록 구성하였다. 이는 채점기에서 요구하는 인터페이스를 유지하기 위한 것이다.

```matlab
actCmd.latCmd.steerAngle = steerAngle;
actCmd.latCmd.yawMoment  = yawMoment;

actCmd.lonCmd.Fx_total = Fx_total;
actCmd.lonCmd.brakeRatio = brakeRatio;

actCmd.verCmd = verCmd;
```

또한 직접 field를 사용하는 main loop에 대응하기 위해 다음 field도 유지하였다.

```matlab
actCmd.steerAngle = steerAngle;
actCmd.yawMoment  = yawMoment;
actCmd.Fx_total   = Fx_total;
actCmd.brakeRatio = brakeRatio;
```

Coordinator에서 brake torque 분배, damping coefficient 강제 생성, yaw moment allocation을 직접 수행하지 않은 이유는 starter 구조와 달라질 경우 실행 오류 또는 감점 가능성이 있기 때문이다.

---

## 4. 시뮬레이션 결과

### 4.1 자동채점 결과

본 설계는 다음 명령으로 검증하였다.

```matlab
run('scripts/run_icc_benchmark.m')
run('scripts/grade.m')
```

최종 자동채점 결과는 다음과 같다.

```text
Quantitative: 45.59 / 70.00
Deductions: -0
```

자동 감점이 없다는 것은 함수 실행 오류, 출력 구조 불일치, 필수 파일 누락으로 인한 감점이 발생하지 않았음을 의미한다.

### 4.2 Benchmark 결과 요약

| 시나리오 | KPI | OFF | ON | 변화율 |
|---|---:|---:|---:|---:|
| A1 DLC | sideSlipMax [deg] | 3.0154 | 2.9730 | -1.4% |
| A1 DLC | LTR_max | 0.8635 | 0.8261 | -4.3% |
| A1 DLC | lateralDevMax [m] | 1.8270 | 1.8658 | +2.1% |
| A3 Step Steer | sideSlipMax [deg] | 1.1138 | 0.9247 | -17.0% |
| A3 Step Steer | LTR_max | 0.4157 | 0.3680 | -11.5% |
| A3 Step Steer | yawRateOvershoot [%] | 2.6997 | 2.9857 | +10.6% |
| A4 Circular | sideSlipMax [deg] | 1.1839 | 1.1734 | -0.9% |
| A4 Circular | LTR_max | 0.0258 | 0.0255 | -0.9% |
| A7 Brake-in-Turn | sideSlipMax [deg] | 30.4776 | 1.9428 | -93.6% |
| A7 Brake-in-Turn | LTR_max | 0.6808 | 0.3298 | -51.6% |
| A7 Brake-in-Turn | tireUtilizationMax | 1.0467 | 0.8805 | -15.9% |
| B1 Straight Braking | stoppingDistance [m] | 72.2992 | 75.7445 | +4.8% |
| B1 Straight Braking | absSlipRMS | 0.7295 | 0.2391 | -67.2% |
| D1 DLC + Braking | sideSlipMax [deg] | 4.9057 | 3.8235 | -22.1% |
| D1 DLC + Braking | LTR_max | 0.8635 | 0.8261 | -4.3% |
| D1 DLC + Braking | tireUtilizationMax | 1.0988 | 1.0488 | -4.5% |

### 4.3 자동채점 항목별 점수

| 시나리오 | KPI | 측정값 | 목표값 | 점수 |
|---|---:|---:|---:|---:|
| A3 | yawRateOvershoot | 2.9857 | 10.0000 | 0.00 / 4 |
| A3 | yawRateRiseTime | 0.1230 | 0.3000 | 4.00 / 4 |
| A3 | yawRateSettling | 1.2270 | 0.8000 | 1.87 / 4 |
| A1 | sideSlipMax | 2.9730 | 3.0000 | 6.00 / 6 |
| A1 | LTR_max | 0.8261 | 0.6000 | 3.12 / 5 |
| A1 | lateralDevMax | 1.8658 | 0.7000 | 0.00 / 4 |
| A4 | understeerGradient | 0.0007 | 0.0030 | 5.00 / 5 |
| A4 | sideSlipMax | 1.1734 | 2.0000 | 5.00 / 5 |
| A7 | sideSlipMax | 1.9428 | 5.0000 | 8.00 / 8 |
| A7 | LTR_max | 0.3298 | 0.7000 | 7.00 / 7 |
| B1 | stoppingDistance | 75.7445 | 66.5000 | 0.00 / 5 |
| B1 | absSlipRMS | 0.2391 | 0.1000 | 0.36 / 5 |
| D1 | sideSlipMax | 3.8235 | 4.0000 | 4.00 / 4 |
| D1 | LTR_max | 0.8261 | 0.6000 | 1.25 / 2 |
| D1 | lateralDevMax | 1.8658 | 1.0000 | 0.00 / 2 |

---

## 5. 시나리오별 분석

### 5.1 A1 Double Lane Change

A1은 급격한 차선 변경 상황에서 path tracking과 횡방향 안정성을 동시에 평가한다. Controller on 상태에서 sideSlipMax는 3.0154 deg에서 2.9730 deg로 감소하였고, LTR_max도 0.8635에서 0.8261로 감소하였다. 이는 ESC와 조향 보정이 차량의 횡방향 안정성 개선에 일부 기여했음을 의미한다.

그러나 lateralDevMax는 1.8270 m에서 1.8658 m로 증가하였다. 이는 본 설계가 path tracking error를 직접 제어하지 않고 yaw rate와 slip angle 중심으로 안정성 제어를 수행했기 때문이다. 즉, 안정성 향상을 위해 yaw moment와 조향 보정이 개입하면서 reference path 추종 성능은 일부 희생되었다.

### 5.2 A3 Step Steer

A3에서는 controller on 상태에서 yawRateRiseTime이 0.2470 s에서 0.1230 s로 개선되었다. 이는 AFS 기반 yaw rate tracking이 yaw response를 빠르게 만든 결과이다.

또한 sideSlipMax는 1.1138 deg에서 0.9247 deg로 감소하였고, LTR_max는 0.4157에서 0.3680으로 감소하였다. 이는 yaw response가 빨라졌음에도 slip angle과 roll stability가 개선되었음을 보여준다.

다만 yawRateOvershoot는 2.6997에서 2.9857로 증가하였다. 이는 빠른 rise time과 overshoot 사이의 trade-off로 해석된다.

### 5.3 A4 Steady-State Circular

A4는 정상상태 원선회 특성을 평가한다. 본 설계에서는 understeerGradient가 0.0007로 측정되어 목표값 0.0030 이하를 만족하였다. sideSlipMax도 1.1734 deg로 목표값 2.0 deg 이하를 만족하였다.

A4에서 controller off와 on의 차이는 크지 않았다. 이는 정상상태 원선회에서는 큰 slip angle이나 급격한 yaw instability가 발생하지 않기 때문이다. 본 제어기는 slip angle이 커질 때 강하게 개입하도록 설계되어 있어, 안정적인 정상상태 선회에서는 불필요한 개입을 하지 않았다.

### 5.4 A7 Brake-in-Turn

A7은 본 설계에서 가장 큰 성능 개선이 나타난 시나리오이다. Controller off 상태에서는 braking 중 sideSlipMax가 30.4776 deg까지 증가하였다. 이는 제동 중 선회 안정성을 크게 잃는 상황이다.

Controller on 상태에서는 sideSlipMax가 1.9428 deg로 감소하였다. 또한 LTR_max는 0.6808에서 0.3298로 감소하였고, tireUtilizationMax도 1.0467에서 0.8805로 감소하였다.

Brake-in-turn에서는 타이어가 종방향 제동력과 횡방향 코너링력을 동시에 발생시켜야 한다. 이때 마찰 한계에 가까워지면 yaw instability와 side slip이 급격히 증가할 수 있다. 본 설계는 abs(beta) > 3 deg 조건에서 yaw moment를 생성하여 slip angle 증가를 억제하였다. 그 결과 A7에서 side slip과 LTR이 크게 개선되었다.

### 5.5 B1 Straight Braking

B1에서는 absSlipRMS가 0.7295에서 0.2391로 감소하였다. 이는 ctrl_longitudinal의 ABS-like modulation이 과도한 slip을 완화하는 데 효과적이었음을 의미한다.

그러나 stoppingDistance는 72.2992 m에서 75.7445 m로 증가하였다. 수정된 채점 기준에서 B1 stoppingDistance 목표값은 66.5 m이지만, 본 설계의 결과는 75.7445 m로 여전히 목표값보다 크기 때문에 해당 항목 점수는 0점이다. 이는 제동력을 보수적으로 제한한 결과이다. 실제 ABS는 wheel slip ratio를 직접 측정하여 최적 slip 부근에서 제동력을 유지하지만, 본 설계에서는 wheel slip ratio가 직접 입력으로 제공되지 않아 감속도 ax 기반으로 제동력을 완화하였다. 따라서 slip 안정성은 개선되었지만 제동거리는 증가하였다.

### 5.6 D1 통합 시나리오

D1은 DLC와 braking이 결합된 통합 시나리오이다. Controller on 상태에서 sideSlipMax는 4.9057 deg에서 3.8235 deg로 감소하였고, tireUtilizationMax는 1.0988에서 1.0488로 감소하였다. 이는 통합 상황에서도 횡방향 안정성 제어가 효과가 있었음을 의미한다.

다만 lateralDevMax는 A1과 마찬가지로 개선되지 않았다. 이는 본 제어기가 path tracking error를 직접 제어하지 않았기 때문이다. 본 설계는 안정성 중심의 ICC 설계이며, path tracking 성능까지 동시에 최적화하는 구조는 아니다.

---

## 6. 분석 및 한계

### 6.1 가장 성공적이었던 부분

가장 성공적이었던 시나리오는 A7 Brake-in-Turn이다. Controller off 상태에서는 side slip이 30.4776 deg까지 증가했지만, controller on 상태에서는 1.9428 deg로 감소하였다. 이는 약 93.6% 개선이다. 또한 LTR_max와 tireUtilizationMax도 함께 감소하였다.

이 결과는 본 설계의 ESC beta-limiter가 제동 중 선회 상황에서 차량 안정성을 확보하는 데 효과적이었음을 보여준다.

### 6.2 부족했던 부분

부족했던 부분은 A1과 D1의 lateralDevMax, 그리고 B1의 stoppingDistance이다. A1과 D1에서는 side slip과 LTR은 개선되었지만 path tracking error는 개선되지 않았다. 이는 본 설계가 path error를 직접 feedback하지 않았기 때문이다.

B1에서는 absSlipRMS는 개선되었지만 stoppingDistance가 증가하였다. 이는 ABS-like modulation이 안정성을 우선하여 제동력을 줄였기 때문이다.

### 6.3 개선 방향

추가 시간이 있다면 다음 개선을 수행할 수 있다.

1. **Path tracking error feedback 추가**
   - 현재 lateral 제어는 yaw rate와 slip angle 중심이다.
   - lateral deviation과 heading error를 포함하면 A1과 D1의 lateralDevMax를 줄일 수 있다.

2. **Wheel slip ratio 기반 ABS 구현**
   - 현재는 ax 기반 ABS-like logic만 사용하였다.
   - wheel speed와 vehicle speed로 slip ratio를 추정하면 B1 성능을 더 개선할 수 있다.

3. **Yaw moment allocation 최적화**
   - 현재 coordinator는 starter 구조 유지를 위해 pass-through 방식이다.
   - 감점 위험이 없는 범위에서 yaw moment를 brake 또는 steering command로 정교하게 분배하면 성능 향상이 가능하다.

4. **마찰계수 기반 gain scheduling**
   - 현재는 속도 기반 gain scheduling만 사용하였다.
   - 노면 마찰계수와 tire utilization을 고려하면 다양한 상황에서 더 안정적인 결과를 얻을 수 있다.

---

## 7. 제출 기준 점검

본 제출물은 채점 기준에서 요구하는 실행 가능성과 인터페이스 유지를 우선적으로 고려하였다.

| 점검 항목 | 반영 여부 |
|---|---|
| ctrl_lateral.m 출력 구조 유지 | 반영 |
| latCmd.steerAngle 유지 | 반영 |
| latCmd.yawMoment 유지 | 반영 |
| ctrl_longitudinal.m 출력 구조 유지 | 반영 |
| lonCmd.Fx_total 유지 | 반영 |
| ctrl_vertical.m 출력 구조 유지 | 반영 |
| verCmd 유지 | 반영 |
| ctrl_coordinator.m에서 불필요한 actuator 재구성 제거 | 반영 |
| grade.m 실행 완료 | 반영 |
| grade_report.json 생성 | 반영 |
| 자동채점 감점 없음 | 반영 |

최종 실행 결과 quantitative score는 45.59 / 70.00이며, deductions는 -0으로 확인되었다. 따라서 실행 오류나 인터페이스 불일치에 의한 자동 감점은 발생하지 않았다.

---

## 8. 결론

본 프로젝트에서는 통합 샤시 제어기를 lateral, longitudinal, vertical, coordinator의 네 부분으로 나누어 설계하였다. 횡방향 제어에서는 yaw rate tracking PID와 beta-limiter 기반 ESC를 결합하여 차량의 slip angle과 roll stability를 개선하였다. 종방향 제어에서는 PI 기반 속도 제어와 ABS-like modulation을 사용하여 제동 중 slip을 완화하였다. 수직방향 제어에서는 skyhook 기반 semi-active damping을 적용하였다.

검증 결과 A7 Brake-in-Turn 시나리오에서 가장 큰 성능 개선을 확인하였다. sideSlipMax는 30.4776 deg에서 1.9428 deg로 감소하였고, LTR_max도 0.6808에서 0.3298로 감소하였다. 이는 본 설계의 ESC beta-limiter가 제동 중 선회 안정성 확보에 효과적이었음을 보여준다.

반면 A1과 D1에서 lateralDevMax가 개선되지 않았고, B1에서 stoppingDistance가 증가한 한계가 있었다. 이는 본 설계가 path tracking 및 최적 ABS 제어보다는 안정성 확보를 우선한 구조였기 때문이다. 향후에는 lateral deviation feedback, wheel slip ratio 기반 ABS, yaw moment allocation 최적화 등을 추가하면 더 높은 점수를 기대할 수 있다.

최종적으로 본 설계는 starter code의 인터페이스를 유지하면서 자동채점 환경에서 정상 실행되었고, quantitative score 45.59 / 70.00 및 deductions -0을 달성하였다.

---

## 9. 참고문헌

[1] ISO 3888-1:2018, Passenger cars - Test track for a severe lane-change manoeuvre.

[2] ISO 4138:2021, Passenger cars - Steady-state circular driving behaviour - Open-loop test methods.

[3] ISO 7401:2011, Road vehicles - Lateral transient response test methods - Open-loop test methods.

[4] ISO 7975:2019, Passenger cars - Braking in a turn - Open-loop test method.

[5] R. Rajamani, Vehicle Dynamics and Control, 2nd ed., Springer, 2012.

[6] J. Y. Wong, Theory of Ground Vehicles, 4th ed., Wiley, 2008.

[7] D. Hrovat, 'Survey of advanced suspension developments and related optimal control applications,' Automatica, vol. 33, no. 10, pp. 1781-1817, 1997.

---

## 부록 A - 사용한 AI 도구

본 프로젝트에서는 ChatGPT를 사용하여 제어기 구조 정리, MATLAB 코드 문법 검토, 보고서 초안 작성, gain tuning 후보 정리에 도움을 받았다. AI가 제안한 값은 그대로 제출하지 않고, run_icc_benchmark.m 및 grade.m을 통해 직접 시뮬레이션한 결과를 기준으로 수정하였다.

사용 예시는 다음과 같다.

- ctrl_lateral의 AFS + ESC 구조 정리
- yaw rate PID, beta-limiter, gain scheduling 설명 정리
- ctrl_longitudinal의 PI + ABS-like modulation 구조 검토
- ctrl_vertical의 skyhook damping logic 설명 정리
- ctrl_coordinator에서 starter 입출력 구조를 유지하도록 코드 구조 검토
- 자동채점 결과를 바탕으로 보고서 표와 분석 문장 작성

최종 적용한 주요 튜닝값은 다음과 같다.

```matlab
steerScale = 0.11;
maxMz = 900;
Kbeta = 18000;
Kr = 250;

CTRL.LON.Kp = 0.35;
CTRL.LON.Ki = 0.03;
```

---

## 부록 B - sim_params.m 변경사항

본 설계에서는 sim_params.m의 plant 구조나 scenario 설정은 변경하지 않았다. 채점기와 starter 구조를 유지하기 위해 주요 변경은 다음 네 개의 controller 파일에 한정하였다.

```text
scripts/control/ctrl_lateral.m
scripts/control/ctrl_longitudinal.m
scripts/control/ctrl_vertical.m
scripts/control/ctrl_coordinator.m
```

sim_params.m 자체의 핵심 파라미터는 수정하지 않았으며, 제어기 내부에서 필요한 경우 CTRL, LIM 구조체의 값을 읽고, 값이 없을 경우 안전한 default 값을 사용하도록 구현하였다.

주요 controller 내부 tuning 값은 다음과 같다.

```matlab
% ctrl_lateral.m
Kp = getFieldSafe(CTRL.LAT, 'Kp', 1.0);
Ki = getFieldSafe(CTRL.LAT, 'Ki', 0.1);
Kd = getFieldSafe(CTRL.LAT, 'Kd', 0.05);

steerScale = 0.11;
maxMz = 900;
maxMzRate = 60000;
Kbeta = 18000;
Kr = 250;

betaTh = deg2rad(3.0);
betaHard = deg2rad(7.0);

% ctrl_longitudinal.m
Kp = getFieldSafe(CTRL.LON, 'Kp', 0.35);
Ki = getFieldSafe(CTRL.LON, 'Ki', 0.03);
m = 1500;
maxAx = getFieldSafe(LIM, 'MAX_AX', 10.0);
maxJerk = getFieldSafe(LIM, 'MAX_JERK', 50.0);

% ctrl_vertical.m
cMin = getFieldSafe(CTRL.VER, 'cMin', 500);
cMax = getFieldSafe(CTRL.VER, 'cMax', 5000);
cNom = getFieldSafe(CTRL.VER, 'cNom', 0.5 * (cMin + cMax));
maxRate = 80000;
```

---

## 부록 C - 재현 방법

본 결과는 다음 명령어로 재현할 수 있다.

```matlab
cd('C:\Users\wswsc\Downloads\integrated-chassis-control-main\integrated-chassis-control-main\icc-project')
run('scripts/utils/init_project.m')
run('scripts/run_icc_benchmark.m')
run('scripts/grade.m')
```

자동채점 결과는 다음 파일에 저장된다.

```text
grade_report.json
```

최종 자동채점 점수는 다음과 같다.

```text
Quantitative: 45.59 / 70.00
Deductions: -0
```
