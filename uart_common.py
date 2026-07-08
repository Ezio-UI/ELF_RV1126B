# uart_comm.py 独立串口模块
import serial
import time

# 串口参数
UART_DEV = "/dev/ttyS5"
BAUDRATE = 115200
SEND_MIN_INTERVAL = 0.5  # 最小500ms发送一次
_last_send_time = 0

# 初始化串口
ser = serial.Serial(
    port=UART_DEV,
    baudrate=BAUDRATE,
    bytesize=8,
    parity='N',
    stopbits=1,
    timeout=0.5
)

def dir_str_to_code(dir_str):
    # 内部direction字符串转协议编码 left=01 / center=02 / right=03
    if dir_str == "left":
        return "01"
    elif dir_str == "center":
        return "02"
    else:
        return "03"

def send_obstacle_data(detect_list):
    """
    detect_list: [(label, dist_m, direction), ...]
    只取最近障碍物，按DIR:xx,DIST:xx cm发送
    """
    global _last_send_time
    now = time.time()
    if (now - _last_send_time) < SEND_MIN_INTERVAL:
        return
    if len(detect_list) == 0:
        return

    # 按距离由近到远排序，取最近
    detect_list.sort(key=lambda x: x[1])
    lab, dist_m, dir_text = detect_list[0]
    dist_cm = round(dist_m * 100)
    dir_code = dir_str_to_code(dir_text)

    send_pkt = f"DIR:{dir_code},DIST:{dist_cm}\n".encode("utf-8")
    ser.write(send_pkt)
    ser.flush()
    print("UART SEND Obstacle:", send_pkt)
    _last_send_time = now

def uart_close():
    ser.close()
