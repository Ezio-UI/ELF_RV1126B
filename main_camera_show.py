# -*- coding: utf-8 -*-   
  
import os  
import time  
import cv2  
import numpy as np  
from rknnlite.api import RKNNLite  
from http.server import BaseHTTPRequestHandler, HTTPServer  
from socketserver import ThreadingMixIn  
from threading import Condition, Thread, Lock  
from queue import Queue  
from concurrent.futures import ThreadPoolExecutor  
from collections import defaultdict  
from uart_common import send_obstacle_data, uart_close

# 导入语音播报模块  
from voice_announcer import SegmentedAnnouncer  
  
# ==================== 配置区 ====================  
MODEL_PATH = './model/tactile_paving.rknn'   
CAMERA_PATH = "/dev/video52"          
PORT = 8080                           
TPEs = 2                            
  
CLASSES = [   
    "car", "dog", "person", "bus", "truck",   
    "green_light", "pole", "sign", "warning_column", "tree",   
    "red_light", "fire_hydrant", "motorcycle", "ashcan", "bicycle",   
    "reflective_cone", "blind_road", "crosswalk", "tricycle", "roadblock"   
]   
  
CONF_THRESH = 0.50                  
NMS_THRESH = 0.45      
MODEL_INPUT_SIZE = (640, 640)   
  
# ==================== 双目原生 640x480 几何空间 ====================  
print("载入 640x480 原生标定参数...")   
calib = np.load("stereo_params.npz")    
mapL1 = calib["mapL1"]  
mapL2 = calib["mapL2"]  
mapR1 = calib["mapR1"]  
mapR2 = calib["mapR2"]  
mtxL = calib["mtx_l"]   
T = calib["T"]   
SCALE_CORRECT = 1.2

# 自动解析基线 T
tx = abs(T[0, 0])
baseline = tx * 1000.0 if tx < 1.0 else tx  # 例如：0.064 -> 64.0mm / 64.5 -> 64.5mm
focal = mtxL[0, 0]                          # 像素焦距

print("✔ 几何解算器配置成功：")
print("   -> 标定图尺寸: {}x{}".format(mapL1.shape[1], mapL1.shape[0]))
print("   -> 像素焦距 focal = {:.2f} px".format(focal))
print("   -> 物理基线 baseline = {:.2f} mm".format(baseline))
  
dist_buffer_lock = Lock()   
tracker_lock = Lock()   
dist_buffer = defaultdict(list)     
buf_len = 10  # 平滑滤波器长度              
  
# ==================== 目标跟踪器 ====================  
class SimpleTracker:   
    def __init__(self):   
        self.next_id = 0  
        self.tracked_boxes = {}   
  
    def update(self, detected_boxes, detected_cls_ids):   
        new_tracked_boxes = {}   
        assigned_ids = []   
        for box, cls_id in zip(detected_boxes, detected_cls_ids):   
            best_id = None  
            best_iou = 0.3  
            for tid, tbox in self.tracked_boxes.items():   
                if tbox[4] != cls_id:   
                    continue  
                iou = self.compute_iou(box, tbox[:4])   
                if iou > best_iou:   
                    best_iou = iou  
                    best_id = tid  
            if best_id is not None and best_id not in assigned_ids:   
                new_tracked_boxes[best_id] = (*box, cls_id)   
                assigned_ids.append(best_id)   
            else:   
                new_tracked_boxes[self.next_id] = (*box, cls_id)   
                assigned_ids.append(self.next_id)   
                self.next_id += 1  
        self.tracked_boxes = new_tracked_boxes  
        return assigned_ids  
  
    @staticmethod  
    def compute_iou(boxA, boxB):   
        xA, yA = max(boxA[0], boxB[0]), max(boxA[1], boxB[1])   
        xB, yB = min(boxA[2], boxB[2]), min(boxA[3], boxB[3])   
        interArea = max(0, xB - xA) * max(0, yB - yA)   
        boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])   
        boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])   
        return interArea / float(boxAArea + boxBArea - interArea + 1e-6)   
  
tracker = SimpleTracker()   
  
# ==================== YOLOv8 中心框解码器 ====================  
def decode_yolo_boxes(valid_boxes):
    if len(valid_boxes) == 0:
        return []
    nms_boxes = []
    for box in valid_boxes:
        b0, b1, b2, b3 = box[0], box[1], box[2], box[3]
        x1 = b0 - b2 / 2.0
        y1 = b1 - b3 / 2.0
        w = b2
        h = b3
        nms_boxes.append([x1, y1, w, h])
    return nms_boxes

# ==================== StereoBM 局部精准 ROI 测距 ====================  
def get_object_distance_roi(rectL, rectR, matcher, x1, y1, x2, y2, obj_id):   
    pad = 12                  
    numDisparities = 128  # 640x480分辨率下，128的搜索能力可支持近至45厘米的测距
    h, w = rectL.shape[:2]
    
    # 构建包含视差搜索空间的 ROI 区域
    cx1 = max(0, x1 - numDisparities - pad)
    cy1 = max(0, y1 - pad)
    cx2 = min(w, x2 + pad)
    cy2 = min(h, y2 + pad)
    
    min_width = numDisparities + 16
    if (cx2 - cx1) < min_width:
        if cx1 == 0:
            cx2 = min(w, cx1 + min_width)
        else:
            cx1 = max(0, cx2 - min_width)
            
    if (cx2 - cx1) < min_width or (cy2 - cy1) < 16:
        return None
        
    cropL = rectL[cy1:cy2, cx1:cx2]
    cropR = rectR[cy1:cy2, cx1:cx2]
    
    grayL = cv2.cvtColor(cropL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(cropR, cv2.COLOR_BGR2GRAY)
    
    disp_crop = matcher.compute(grayL, grayR)
    
    # 映射回目标框在局部视差图中的内部坐标
    rx1 = max(0, x1 - cx1)
    ry1 = max(0, y1 - cy1)
    rx2 = min(disp_crop.shape[1] - 1, x2 - cx1)
    ry2 = min(disp_crop.shape[0] - 1, y2 - cy1)
    
    if rx2 <= rx1 or ry2 <= ry1:
        return None
        
    roi = disp_crop[ry1:ry2, rx1:rx2]
    valid = roi[(roi > 0) & (roi < numDisparities * 16)]
    
    if len(valid) < 15:
        return None
        
    # 取中位数视差，过滤边界噪声
    avg_disp = np.median(valid) / 16.0
    if avg_disp < 0.5:   
        return None
        
    # 物理三角测距: Z = (f * B) / d
    dist_mm = (focal * baseline) / avg_disp
    dist_cm = dist_mm / 10.0
    
    real_dist_cm = dist_cm * SCALE_CORRECT
    
    with dist_buffer_lock: 
        dist_buffer[obj_id].append(real_dist_cm)  # 存入修正后的距离值
        if len(dist_buffer[obj_id]) > buf_len:   
            dist_buffer[obj_id].pop(0)   
        return np.mean(dist_buffer[obj_id])

  
# ==================== 服务器画面流传输 ====================  
class StreamingOutput(object):   
    def __init__(self):   
        self.frame = None  
        self.condition = Condition()   
  
    def write(self, frame):   
        with self.condition:   
            self.frame = frame  
            self.condition.notify_all()   
  
class StreamingHandler(BaseHTTPRequestHandler):   
    def do_GET(self):   
        if self.path == '/':   
            self.send_response(200)   
            self.send_header('Age', 0)   
            self.send_header('Cache-Control', 'no-cache, private')   
            self.send_header('Pragma', 'no-cache')   
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')   
            self.end_headers()   
            try:   
                while True:   
                    with streaming_output.condition:   
                        streaming_output.condition.wait()   
                        frame = streaming_output.frame  
                    self.wfile.write(b'--FRAME\r\n')   
                    self.send_header('Content-Type', 'image/jpeg')   
                    self.send_header('Content-Length', len(frame))   
                    self.end_headers()   
                    self.wfile.write(frame)   
                    self.wfile.write(b'\r\n')   
            except Exception:   
                pass  
  
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):   
    allow_reuse_address = True  
    daemon_threads = True  
  
streaming_output = StreamingOutput()   
  
# ==================== 摄像头驱动线程 ====================  
class CameraCapture:   
    def __init__(self, path):   
        print("📷 正在初始化物理拼接摄像头: {} ...".format(path))  
        self.cap = cv2.VideoCapture(path, cv2.CAP_V4L2)  
        if not self.cap.isOpened():  
            self.cap = cv2.VideoCapture(path)  
            
        if not self.cap.isOpened():  
            print("\n物理节点无法驱动！")  
            os._exit(-1)  
            
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))   
        # 原生输出尺寸：双目合并为 1280x480，单目即 640x480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)   
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)   
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  
        
        self.ret, self.frame = False, None  
        self.running = True  
        self.lock = Lock()   
        self.thread = Thread(target=self._update, args=())   
        self.thread.daemon = True  
        self.thread.start()   
  
    def _update(self):   
        while self.running:   
            ret, frame = self.cap.read()   
            if ret:   
                with self.lock:   
                    self.ret = ret  
                    self.frame = frame  
            time.sleep(0.005) 
  
    def read(self):   
        with self.lock:   
            return self.ret, self.frame  
  
    def release(self):   
        self.running = False  
        self.cap.release()   
  
# ==================== NPU 推理流水线 ====================  
def worker_inference(rknn_lite, matcher, frame):   
    actual_h, actual_w = frame.shape[:2]
    
    # 物理无损切割左右眼 (每个均为 640x480)
    rawL = frame[:, :640]   
    rawR = frame[:, 640:]   
      
    # 直接套用标定矩阵重映射，极线物理精细对齐
    rectL = cv2.remap(rawL, mapL1, mapL2, cv2.INTER_LINEAR)   
    rectR = cv2.remap(rawR, mapR1, mapR2, cv2.INTER_LINEAR)   
  
    # 仅在送入 NPU 推理前，做 640x640 缩放
    img_bgr = cv2.resize(rectL, MODEL_INPUT_SIZE)   
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)   
    img_input = np.ascontiguousarray(
        np.expand_dims(img_rgb, axis=0), dtype=np.uint8
    )   
  
    outputs = rknn_lite.inference(inputs=[img_input])   
    show_img = rectL.copy()   
  
    detected_targets = []    
  
    if outputs is not None and len(outputs) > 0:   
        raw_out = outputs[0]   
        if len(raw_out.shape) == 3 and raw_out.shape[1] > raw_out.shape[2]:   
            raw_out = np.transpose(raw_out, (0, 2, 1))   
        output = raw_out[0]    
  
        scores = output[4:24, :]                        
        class_ids = np.argmax(scores, axis=0)            
        confidences = np.max(scores, axis=0)             
  
        mask = confidences > CONF_THRESH  
        if np.any(mask):   
            valid_boxes = output[0:4, mask].T            
            valid_confs = confidences[mask]              
            valid_class_ids = class_ids[mask]            
  
            nms_boxes = decode_yolo_boxes(valid_boxes)
            nms_confs = valid_confs.astype(float).tolist()   
  
            try:   
                indices = cv2.dnn.NMSBoxes(
                    nms_boxes, nms_confs, CONF_THRESH, NMS_THRESH
                )   
                if len(indices) > 0:   
                    indices = indices.flatten()   
            except AttributeError:   
                indices = []   
  
            # 检测坐标缩放因子：由 NPU 尺寸 640x640 映射回实际的 640x480
            scale_x_det = 640.0 / MODEL_INPUT_SIZE[0]         # = 1.0  
            scale_y_det = 480.0 / MODEL_INPUT_SIZE[1]         # = 0.75  
  
            local_detected_boxes = []   
            local_detected_cls = []   
            pre_render_list = []   
  
            for idx in indices:   
                box = nms_boxes[idx]   
                rx1 = max(0, int(box[0] * scale_x_det))   
                ry1 = max(0, int(box[1] * scale_y_det))   
                rx2 = min(640 - 1, int((box[0] + box[2]) * scale_x_det))   
                ry2 = min(480 - 1, int((box[1] + box[3]) * scale_y_det))   
                
                rw = rx2 - rx1
                rh = ry2 - ry1
  
                class_id = int(valid_class_ids[idx])   
                conf = float(valid_confs[idx])   
                label = CLASSES[class_id] if class_id < len(CLASSES) else "ID"
  
                if rw > int(640 * 0.9) or rh > int(480 * 0.9):
                    continue
                if rw < 10 or rh < 10:   
                    continue  
  
                local_detected_boxes.append([rx1, ry1, rx2, ry2])   
                local_detected_cls.append(class_id)   
                pre_render_list.append((rx1, ry1, rx2, ry2, label, conf))   
  
            if len(local_detected_boxes) > 0:   
                with tracker_lock:   
                    obj_ids = tracker.update(
                        local_detected_boxes, local_detected_cls
                    )   
  
                for (rx1, ry1, rx2, ry2, label, conf), obj_id in zip(
                    pre_render_list, obj_ids
                ):   
                    # 在 640x480 原生对齐图上运行局部深度估计
                    distance = get_object_distance_roi(
                        rectL, rectR, matcher, rx1, ry1, rx2, ry2, obj_id
                    )   
                    dist_text = ""   
                      
                    if distance is not None:   
                        distance_m = distance / 100.0  
                          
                        box_cx = rx1 + (rx2 - rx1) / 2.0  
                        if box_cx < 640 / 3.0:  
                            direction = "left"  
                            dir_tag = " [L]"  
                        elif box_cx > 640 * 2.0 / 3.0:  
                            direction = "right"  
                            dir_tag = " [R]"  
                        else:  
                            direction = "center"  
                            dir_tag = " [C]"  
                          
                        dist_text = " {:.1f}cm{}".format(distance, dir_tag)   
                        detected_targets.append((label, distance_m, direction))   
  
                    cv2.rectangle(show_img, (rx1, ry1), (rx2, ry2), (0,255,0), 2)   
                    label_text = "{}{}".format(label, dist_text)
                    (text_w, text_h), _ = cv2.getTextSize(
                        label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1
                    )   
                      
                    text_back_y1 = ry1  
                    text_back_y0 = ry1 - text_h - 6  
                    if text_back_y0 < 0:    
                        text_back_y0 = ry1 + 2  
                        text_back_y1 = ry1 + text_h + 8  
                        text_y = text_back_y1 - 4  
                    else:   
                        text_y = text_back_y1 - 4  
  
                    cv2.rectangle(
                        show_img, 
                        (rx1, text_back_y0), 
                        (rx1 + text_w + 8, text_back_y1), 
                        (0, 255, 0), 
                        -1
                    )   
                    cv2.putText(
                        show_img, 
                        label_text, 
                        (rx1 + 4, text_y),   
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.45, 
                        (0, 0, 0), 
                        1, 
                        cv2.LINE_AA
                    )   
  
    diag_text = "FPS Mode | Size: {}x{} | Focal: {:.1f}px".format(
        640, 480, focal
    )
    cv2.putText(
        show_img, 
        diag_text, 
        (10, 480 - 10), 
        cv2.FONT_HERSHEY_SIMPLEX, 
        0.45, 
        (0, 0, 255), 
        1, 
        cv2.LINE_AA
    )
  
    # 放大至网页便于观看的尺寸 (800x600，仅用于流输出)
    show_img_large = cv2.resize(
        show_img, (800, 600), interpolation=cv2.INTER_LINEAR
    )
  
    _, jpeg = cv2.imencode(
        '.jpg', show_img_large, [int(cv2.IMWRITE_JPEG_QUALITY), 80]
    )   
    return jpeg.tobytes(), detected_targets   
  
# ==================== RKNN 物理实例化 ====================  
def initRKNN(rknnModel, instance_id):   
    rknn_lite = RKNNLite()   
    if rknn_lite.load_rknn(rknnModel) != 0 or rknn_lite.init_runtime() != 0:   
        print(f"RV1126 NPU 初始化失败！")   
        exit(-1)   
        
    # 重算高清 1280x720 坐标下的块匹配
    matcher = cv2.StereoBM_create(numDisparities=128, blockSize=15)   
    matcher.setMinDisparity(0)   
    matcher.setUniquenessRatio(8)   
    matcher.setSpeckleWindowSize(100)   
    matcher.setSpeckleRange(2)   
        
    return rknn_lite, matcher  
  
class RKNNPoolExecutor:   
    def __init__(self, rknnModel, TPEs, func):   
        self.TPEs = TPEs  
        self.queue = Queue()   
        self.rknnPool = [initRKNN(rknnModel, i) for i in range(TPEs)]   
        self.pool = ThreadPoolExecutor(max_workers=TPEs)   
        self.func = func  
        self.num = 0  
        self.is_running = True   
  
    def put(self, frame):   
        if not self.is_running:   
            return  
        rknn_lite, matcher = self.rknnPool[self.num % self.TPEs]   
        self.queue.put(self.pool.submit(self.func, rknn_lite, matcher, frame))   
        self.num += 1  
  
    def get(self):   
        if self.queue.empty():   
            return None, False  
        future = self.queue.get()   
        try:   
            return future.result(), True  
        except Exception:   
            return None, False  
  
    def release(self):   
        self.is_running = False  
        self.pool.shutdown(wait=True)   
        for rknn_lite, _ in self.rknnPool:   
            rknn_lite.release()   
  
# ==================== 系统管路死循环 ====================  
def npu_pipeline_loop():   
    announcer = SegmentedAnnouncer()   
  
    pool = RKNNPoolExecutor(MODEL_PATH, TPEs, worker_inference)   
    cam = CameraCapture(CAMERA_PATH)   
    time.sleep(1.5)    
  
    for _ in range(TPEs):   
        ret, frame = cam.read()   
        if ret:   
            pool.put(frame)   
  
    print("NPU 高帧避障系统恢复运转")   
  
    try:   
        while True:   
            result, success = pool.get()   
            if not success or result is None:   
                time.sleep(0.002)   
                continue  
              
            jpeg_bytes, detections = result  
              
            valid_detections = [d for d in detections if d[0] != "sign"]  
              
            if len(valid_detections) > 0:   
                HAZARD_WEIGHTS = {  
                    "person": 0.0,  
                    "red_light": 0.1,    
                    "dog": 0.2,  
                    "motorcycle": 0.3, "bicycle": 0.3, "tricycle": 0.3,  
                    "car": 0.4, "bus": 0.4, "truck": 0.4,  
                      
                    "roadblock": 1.0, "reflective_cone": 1.0, 
                    "warning_column": 1.0,  
                    "fire_hydrant": 1.1, "ashcan": 1.1, "pole": 1.2, 
                    "tree": 1.3,  
                      
                    "green_light": 2.0, "blind_road": 2.1, "crosswalk": 2.2  
                }  
  
                def get_priority(det):   
                    label, dist, direction = det  
                    if label in ["green_light", "blind_road", "crosswalk"]:   
                        zone_priority = 2    
                    else:   
                        if dist < 2.0:   
                            zone_priority = 0      
                        elif dist < 5.0:   
                            zone_priority = 1      
                        else:   
                            zone_priority = 2      
                      
                    hazard_val = HAZARD_WEIGHTS.get(label, 1.0)  
                    return (zone_priority, hazard_val, dist)  
                  
                valid_detections.sort(key=get_priority)   
                urgent_label, urgent_dist, urgent_dir = valid_detections[0]   
                announcer.play_alert(urgent_label, urgent_dist, urgent_dir)   
            send_obstacle_data(detections)
            ret, next_frame = cam.read()   
            if ret:   
                pool.put(next_frame)   
  
            streaming_output.write(jpeg_bytes)   
            time.sleep(0.001)  
            
    finally:   
        print("正在平滑停止进程...")   
        uart_close()
        cam.release()   
        pool.release()   
  
if __name__ == '__main__':   
    t = Thread(target=npu_pipeline_loop)   
    t.daemon = True  
    t.start()   
  
    server_address = ('0.0.0.0', PORT)   
    server = ThreadedHTTPServer(server_address, StreamingHandler)   
    print(f"==================================================")   
    print(f" RV1126 高帧避障网页运行于：")   
    print(f" http://开发板实际IP地址:{PORT}")   
    print(f"==================================================")   
    try:   
        server.serve_forever()   
    except KeyboardInterrupt:   
        print("服务已手动中断。")
