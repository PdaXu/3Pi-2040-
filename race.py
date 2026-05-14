from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time
import _thread

# =======================================================
# 1. 初始化和配置
# =======================================================
display = robot.Display()
motors = robot.Motors()
line_sensors = robot.LineSensors()
bump_sensors = robot.BumpSensors()
buzzer = robot.Buzzer()
yellow_led = robot.YellowLED()
button_a = robot.ButtonA()

# 选择版本并设置通用参数
edition = editions.select()
if edition == "Standard":
    MAX_SPEED_LF = 6000     # Line Follower 最大速度
    MAX_SPEED_LK = 3000     # Lane Keeping 最大速度
    MAX_SPEED_WB = 1500     # Wall Bumper 避障速度
    CALIBRATION_SPEED = 1000
    CALIBRATION_COUNT = 100
    TURN_TIME_WB = 230      # Wall Bumper 避障转弯时间 (ms)
elif edition == "Turtle":
    MAX_SPEED_LF = 2000
    MAX_SPEED_LK = 2000
    MAX_SPEED_WB = 1500
    CALIBRATION_SPEED = 1125
    CALIBRATION_COUNT = 100
    TURN_TIME_WB = 270
elif edition == "Hyper":
    MAX_SPEED_LF = 6000
    MAX_SPEED_LK = 4000
    MAX_SPEED_WB = 1125
    CALIBRATION_SPEED = 1000
    CALIBRATION_COUNT = 100
    TURN_TIME_WB = 100
    motors.flip_left(True)
    motors.flip_right(True)
#音乐播放
song1 = "t96 l16 ms v15 " + \
    "O4 a O5 e a e O4 f O5 c f c " + \
    "O4 g O5 d g d " + \
    "O4 c g O5 c g " + \
    "O4 a O5 e a e O4 f O5 c f c " + \
    "O4 g O5 d g d " + \
    "O4 c " + \
    "l8 O4 g O5 c " + \
    "l8 O5 d e" +\
     "O5 d " + \
    "l8 O5 c O5 c " + \
    "l16 O4 c O4 e O4 f O4 a " + \
    "l8 O4 g O5 c " + \
    "l8 O5 d O5 e " + \
    \
    "O5 d O5 c O5 d O5 e O5 e O5 e O4 e " + \
    \
    "l8 O4 g O5 c " + \
    "l8 O5 d O5 e " + \
    \
    "O5 d " + \
    "l8 O5 c O5 c " + \
    "l16 O4 c O4 e O4 f O4 a " + \
    "l8 O4 g O5 c " + \
    "l8 O5 d O5 e " + \
    \
    "O5 d O5 c O5 d O5 e " + \
    "O5 g O5 g " + \
    "O5 e O4 e O5 e O5 f" + \
    "O5 g l8 O5 g O5 g O5 d O5 g O5 g l8 O5 e O5 c O4 c l8 O5 e O5 f " + \
    "O5 g l8 O5 g O5 g O4 b O5 g O5 g l8 O5 e O5 c O4 c l8 O5 c O5 d " + \
    "O5 e l8 O5 e O5 e O4 a O5 e O5 e O5 a O5 d l8 O5 e O5 d " + \
    "O5 c O5 d O4 d O4 g l8 O4 a O4 a O4 a O4 b l8 O4 g O5 c l8 O5 d O5 e " + \
    "O5 d l8 O5 c O5 c O4 c O4 e O4 f O4 a l8 O4 g O5 c l8 O5 d O5 e " + \
    "O5 d l8 O5 c O5 d l8 O5 e O5 e O5 e O4 e l8 O4 g O5 c l8 O5 d O5 e"
song2 = "t96 l16 ms v15 " + \
    "l8 O5 c O5 d O5 e O5 g O5 g l8 O5 a O5 a O5 g O5 e O5 c r16 l8 O5 d " + \
    "O5 e O5 e O5 d O5 d O5 c " + \
    "O5 e O5 g O5 g l8 O5 a O5 a O5 g O5 e O5 c r16 l8 O5 d " + \
    "O5 e O5 e O5 d O5 c O5 d " + \
    "O5 e O5 g O5 g l8 O5 a O5 a O5 g O5 e O5 c r16 l8 O5 d " + \
    "O5 e O5 e O5 d O5 d O5 c " + \
    "O5 f O5 f O5 f O5 a O5 g l8 O5 g O5 e O5 d O5 e " + \
    "O5 g O5 g l8 O5 a O5 a O5 g O5 e O5 c r16 l8 O5 d " + \
    "O5 e O5 e O5 d O5 d O5 c O5 d"

# PID / 控制状态变量
# Line Follower 变量
LF_sensor_history = [[0]*5 for _ in range(3)]
LF_sensor_index = 0
LF_integral = 0
LF_line_lost_counter = 0
LF_last_known_direction = 0
LF_last_p = 0
LF_p = 0
LF_max_acceleration = 1000

# Lane Keeping 变量
LK_sensor_history = [[0]*5 for _ in range(3)]
LK_sensor_index = 0
LK_BASE_SPEED_RATIO = 0.65
LK_base_speed = int(MAX_SPEED_LK * LK_BASE_SPEED_RATIO)
LK_Kp = 0.35
LK_Ki = 0.0008
LK_Kd = 6.0
LK_pid_integral = 0.0
LK_pid_last_error = 0.0
LK_pid_integral_limit = MAX_SPEED_LK * 5
LK_LINE_THRESHOLD = 360

# 通用状态变量
last_left_speed = 0
last_right_speed = 0
max_acceleration = 2000

# 比赛状态
STATE_WAIT = 0
STATE_LF_CALIBRATE = 1
STATE_LF_RUN = 2
STATE_LF_COLLISION = 3
STATE_DRIVE_STRAIGHT = 4 # 新增状态：直线行走
STATE_LK_RUN = 5         # Lane Keeping 阶段
competition_state = STATE_WAIT

# 控制线程标志
run_control_thread = True
run_motors = False

# 直线行走配置
STRAIGHT_DRIVE_DURATION_MS = 400 # 避障后直线行走的时间 (ms)

# =======================================================
# 2. 辅助函数 (加速度/滤波)
# =======================================================

def set_motors_with_accel_limit(left, right, max_accel):
    """ 限制加速度地设置电机速度 """
    global last_left_speed, last_right_speed
    
    # 限制加速度
    left_accel = max(-max_accel, min(max_accel, left - last_left_speed))
    right_accel = max(-max_accel, min(max_accel, right - last_right_speed))
    
    left = last_left_speed + left_accel
    right = last_right_speed + right_accel
    
    motors.set_speeds(int(left), int(right))
    last_left_speed = left
    last_right_speed = right

def read_filtered_sensors(sensor_history, sensor_index):
    """ 中值滤波读取 5 个传感器 """
    raw = line_sensors.read_calibrated()[:]
    sensor_history[sensor_index] = raw
    sensor_index = (sensor_index + 1) % 3

    filtered = []
    for i in range(5):
        vals = [sensor_history[j][i] for j in range(3)]
        filtered.append(sorted(vals)[1])
    return filtered, sensor_index

# =======================================================
# 3. Line Follower 逻辑
# =======================================================

def LF_calculate_line_position(sensors):
    active_sensors = []
    for i, value in enumerate(sensors):
        if value > 400 and value < 1100:
            active_sensors.append((i, value))
    
    if not active_sensors:
        return None
    
    total_weight = sum(weight for _, weight in active_sensors)
    if total_weight == 0:
        return 2000
    
    position = sum(pos * 1000 * weight for pos, weight in active_sensors) / total_weight
    return position

def LF_get_dynamic_pid_params(p_val):
    abs_p = abs(p_val)
    if abs_p < 500:
        return 40, 1800, 0.1
    elif abs_p < 1500:
        return 120, 2500, 0.15
    else:
        return 150, 3000, 0.2

def LF_adaptive_speed_control(p_val, d_val):
    curvature = abs(p_val) + abs(d_val) * 10
    
    if curvature < 500:
        return MAX_SPEED_LF
    elif curvature < 1500:
        return MAX_SPEED_LF * 0.8
    else:
        return MAX_SPEED_LF * 0.5

def LF_handle_line_lost(p_val, last_direction):
    global LF_line_lost_counter
    
    if LF_line_lost_counter == 0:
        last_direction = -1 if p_val < 0 else 1
    
    LF_line_lost_counter += 1
    
    if LF_line_lost_counter < 10:
        return last_direction * MAX_SPEED_LF * 0.6, -last_direction * MAX_SPEED_LF * 0.6, last_direction
    else:
        return MAX_SPEED_LF * 0.4, -MAX_SPEED_LF * 0.4, last_direction

def follow_line_step():
    global LF_sensor_history, LF_sensor_index, LF_integral, LF_line_lost_counter, LF_last_known_direction
    global LF_last_p, LF_p, run_motors
    
    # 1. 读取并滤波传感器
    line, LF_sensor_index = read_filtered_sensors(LF_sensor_history, LF_sensor_index)
    line_sensors.start_read()

    # 2. 线路位置计算
    line_pos = LF_calculate_line_position(line)
    
    if line_pos is None:  # 线路丢失
        left_speed, right_speed, LF_last_known_direction = LF_handle_line_lost(LF_p, LF_last_known_direction)
        if run_motors:
            set_motors_with_accel_limit(left_speed, right_speed, LF_max_acceleration)
        return
    else:
        LF_line_lost_counter = 0
        LF_p = line_pos - 2000
    
    # 3. 动态PID参数
    kp, kd, ki = LF_get_dynamic_pid_params(LF_p)
    
    # 4. PID计算
    d = LF_p - LF_last_p
    LF_integral = LF_integral * 0.9 + LF_p
    LF_integral = max(-1000, min(1000, LF_integral))
    
    pid = LF_p * kp + d * kd + LF_integral * ki
    
    # 5. 自适应速度
    current_max_speed = LF_adaptive_speed_control(LF_p, d)
    
    # 6. 电机速度计算
    min_speed = current_max_speed * 0.3
    left = max(min_speed, min(current_max_speed, current_max_speed + pid))
    right = max(min_speed, min(current_max_speed, current_max_speed - pid))
    
    if run_motors:
        set_motors_with_accel_limit(left, right, LF_max_acceleration)
    else:
        motors.off()
    
    LF_last_p = LF_p

# =======================================================
# 4. Lane Keeping 逻辑
# =======================================================

def lane_keeping_step():
    global LK_sensor_history, LK_sensor_index, LK_pid_integral, LK_pid_last_error
    global run_motors, LK_base_speed

    # 1) 读取并滤波传感器
    sensor_values, LK_sensor_index = read_filtered_sensors(LK_sensor_history, LK_sensor_index)
    line_sensors.start_read()

    # A) 【紧急情况判断】：如果前方几乎全黑 → 原地左转
    black_count = sum(1 for v in sensor_values if  v > LK_LINE_THRESHOLD )

    if black_count >= 3:
        turn_speed = int(MAX_SPEED_LK * 0.2)
        turn_time = 150

        if run_motors:
            # 停止
            motors.off()
            time.sleep_ms(20)
            # 原地左转
            motors.set_speeds(-turn_speed, turn_speed)
            time.sleep_ms(turn_time)
            # 停顿
            motors.off()
            time.sleep_ms(80)
        
        # 退出，等待下一循环
        return

    # 2) 计算左右黑线强度
    left_intensity = (sensor_values[0] + sensor_values[1]) / 2.0
    right_intensity = (sensor_values[3] + sensor_values[4]) / 2.0

    # 3) 误差 = 右 - 左
    error = right_intensity - left_intensity

    # 4) PID 控制
    LK_pid_integral += error
    LK_pid_integral = max(-LK_pid_integral_limit, min(LK_pid_integral_limit, LK_pid_integral))

    derivative = error - LK_pid_last_error
    LK_pid_last_error = error

    steering = LK_Kp * error + LK_Ki * LK_pid_integral + LK_Kd * derivative

    # 5) 左右速度
    left_speed_cmd = LK_base_speed - steering
    right_speed_cmd = LK_base_speed + steering

    # 6) 限幅并输出电机
    if run_motors:
        set_motors_with_accel_limit(left_speed_cmd, right_speed_cmd, max_acceleration)
    else:
        motors.off()
        
# =======================================================
# 5. 核心控制线程 (状态机)
# =======================================================
def competition_control_thread():
    global competition_state, run_motors, run_control_thread
    global LF_p 
    
    # 新增直线行走计时器
    straight_drive_end_time = 0

    while run_control_thread:

        if competition_state == STATE_LF_RUN:

            # 防止巡线过程中被其他线程意外改状态
            if competition_state != STATE_LF_RUN:
                continue

            # **慢速巡线**：持续运行 line_follower 逻辑
            follow_line_step()
            
            # 碰撞检测 (流程图：左侧碰撞 (调用 wall_bumper) 检测到)
            bump_sensors.read()
            if bump_sensors.left_is_pressed():
                competition_state = STATE_LF_COLLISION
                
        elif competition_state == STATE_LF_COLLISION:
            # **避障动作** (流程图：停止巡线 -> 原地右转 (避开障碍))
            run_motors = False
            motors.off()
            yellow_led.on()

            buzzer.play("a32")
            
            # 原地右转 (避障)
            motors.set_speeds(MAX_SPEED_WB, -MAX_SPEED_WB)
            time.sleep_ms(TURN_TIME_WB)
            
            motors.off()
            buzzer.play("b32")
            yellow_led.off()
            
            # **切换到直线行走状态**
            straight_drive_end_time = time.ticks_ms() + STRAIGHT_DRIVE_DURATION_MS
            competition_state = STATE_DRIVE_STRAIGHT

            #buzzer.play_in_background(song1)
            
        elif competition_state == STATE_DRIVE_STRAIGHT:


            # **直线行走**
            if time.ticks_ms() < straight_drive_end_time:
                # 直线向前行驶
                motors.set_speeds(MAX_SPEED_WB * 0.8, MAX_SPEED_WB * 0.8) 
            else:
                # 行走结束，进入 Lane Keeping 模式
                motors.off()
                
                # 重置 LK PID 状态
                global LK_pid_integral, LK_pid_last_error
                LK_pid_integral = 0.0
                LK_pid_last_error = 0.0
                
                # 切换到 Lane Keeping 阶段
                # 流程图：调用 lane_keeping: 自检 (1 秒检查)
                competition_state = STATE_LK_RUN
                
                # 流程图：启动 lane_keeping 正常模式 (由主循环在 1 秒检查后开启)
                # **注意：此处不立即设置 run_motors = True，留给主循环处理 1 秒检查**
                buzzer.play_in_background(song1)
                
        elif competition_state == STATE_LK_RUN:
            # 跑道保持模式：持续运行 lane_keeping 逻辑
            lane_keeping_step()

        else:
            # 其他状态（WAIT, CALIBRATE, etc.）
            time.sleep_ms(10)
            
# 启动核心控制线程
_thread.start_new_thread(competition_control_thread, ())

# =======================================================
# 6. 主循环 (显示/按键/状态机流程控制)
# =======================================================
def show_status():
    display.fill(0)
    display.text("COMPETITION START", 0, 0)
    
    if competition_state == STATE_WAIT:
        display.text("Press A to Start", 0, 10)
    elif competition_state == STATE_LF_CALIBRATE:
        display.text("Calibrating...", 0, 10)
    elif competition_state == STATE_LF_RUN:
        display.text("Mode: Line Follower", 0, 10)
        display.text(f"P: {int(LF_p)}", 0, 20)
        display.text("Bump: L/R", 0, 30)
        display.fill_rect(50, 30, 8, 8, 1 if bump_sensors.left_is_pressed() else 0)
        display.fill_rect(65, 30, 8, 8, 1 if bump_sensors.right_is_pressed() else 0)
    elif competition_state == STATE_LF_COLLISION:
        display.text("LEFT BUMP! Turning...", 0, 10)
    elif competition_state == STATE_DRIVE_STRAIGHT:
        display.text("Drive Straight...", 0, 10)
    elif competition_state == STATE_LK_RUN:
        display.text("Mode: Lane Keeping", 0, 10)
        display.text(f"I: {int(LK_pid_integral)}", 0, 20)
        display.text(f"Running: {run_motors}", 0, 30)
    
    display.show()


# --- 启动前准备 ---
display.fill(0)
display.text("Competition Mode", 0, 0)
display.text("Place on line.", 0, 20)
display.text("Press A to start.", 0, 30)
display.show()

# ---------------------------
# 启动等待：按 A → 右碰撞 → 2秒启动
# ---------------------------

# 等待按 A 按钮
display.fill(0)
display.text("Press A to arm", 0, 0)
display.show()

while not button_a.check():
    time.sleep_ms(20)

# 防止按键抖动
time.sleep_ms(300)

# 等右侧碰撞触发
display.fill(0)
display.text("Hit bumper", 0, 0)
display.text("to start...", 0, 10)
display.show()

bump_sensors.calibrate()

time.sleep_ms(1000)

while True:
    bump_sensors.read()
    if bump_sensors.right_is_pressed () or bump_sensors.left_is_pressed ():
        buzzer.play("b32")
        break
    time.sleep_ms(20)

# 撞击确认 & 等待 2 秒
display.fill(0)
display.text("Starting in 2s...", 0, 0)
display.show()

time.sleep_ms(2000)



# ---------------------------
# 正式开始
# ---------------------------
display.fill(0)
display.show()
time.sleep_ms(500)

buzzer.play_in_background(song2)


# --- 传感器校准 (Line Follower / Wall Bumper) ---
competition_state = STATE_LF_CALIBRATE
line_sensors.reset_calibration()

# Line Follower 校准
motors.set_speeds(CALIBRATION_SPEED, -CALIBRATION_SPEED)
for i in range(CALIBRATION_COUNT//4):
    line_sensors.calibrate()
motors.off()
time.sleep_ms(150)

motors.set_speeds(-CALIBRATION_SPEED, CALIBRATION_SPEED)
for i in range(CALIBRATION_COUNT//2):
    line_sensors.calibrate()
motors.off()
time.sleep_ms(150)

motors.set_speeds(CALIBRATION_SPEED, -CALIBRATION_SPEED)
for i in range(CALIBRATION_COUNT//4):
    line_sensors.calibrate()
motors.off()
time.sleep_ms(200)

bump_sensors.calibrate() # 校准碰撞传感器
time.sleep_ms(500)


# --- 流程图第二步：调用 line_follower: 巡线自检 (1 秒运行检查) ---
competition_state = STATE_LF_RUN
run_motors = True # 开启电机

start_time = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), start_time) < 1000:
    show_status()
    # 如果在这个阶段检测到碰撞，则状态会切换，提前进入避障
    if competition_state != STATE_LF_RUN:
        break
    time.sleep_ms(10)

# --- 流程图第三步：慢速巡线 (主循环持续执行) ---
# 此时 competition_state 仍为 STATE_LF_RUN (除非发生碰撞)


# --- 主循环：持续显示和处理按键 ---
last_update_ms = time.ticks_ms()
lk_self_check_start = 0
lk_self_check_done = False

while True:
    t = time.ticks_ms()
    
    # **特殊处理：Lane Keeping 1 秒自检**
    if competition_state == STATE_LK_RUN and not lk_self_check_done:
        # 在 STATE_LK_RUN 启动时开始计时
        if lk_self_check_start == 0:
            # 刚进入 LK_RUN 状态
            run_motors = False # 确保电机在 1s 检查开始时不立即转动（自检含义）
            lk_self_check_start = t
            
        elif time.ticks_diff(t, lk_self_check_start) >= 1000:
            # 流程图：启动 lane_keeping 正常模式
            run_motors = True
            lk_self_check_done = True
            lk_self_check_start = 0
    
    # 更新显示
    if time.ticks_diff(t, last_update_ms) > 100:
        last_update_ms = t
        show_status()

    # 按键 A 停止所有电机，并进入等待状态
    if button_a.check():
        run_motors = False
        motors.off()
        
        # 停止所有逻辑
        run_control_thread = False
        
        # 等待按键松开
        while button_a.check():
            time.sleep_ms(50)
            
        display.fill(0)
        display.text("STOPPED", 0, 0)
        display.text("Power cycle to restart", 0, 20)
        display.show()
        
        break

    time.sleep_ms(5)