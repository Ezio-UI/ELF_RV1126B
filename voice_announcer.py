# -*- coding: utf-8 -*- 

import os
import time
import wave
import subprocess
import threading
import numpy as np

print("2. 基础库加载成功。") 

ALSA_DEVICE = ''  
CACHE_DIR = "./voice_cache" 
TEMP_PLAY_WAV = "./yuyinmodel/voice_temp.wav"  

os.makedirs(CACHE_DIR, exist_ok=True) 
os.makedirs(os.path.dirname(TEMP_PLAY_WAV), exist_ok=True) 

LEVELS = { 
    "danger": ("危险", "lvl_danger.wav"), 
    "warning": ("注意", "lvl_warning.wav"), 
    "notice": ("", None) 
} 

DIRECTIONS = { 
    "left": ("左前方", "dir_left.wav"), 
    "center": ("正前方", "dir_center.wav"), 
    "right": ("右前方", "dir_right.wav") 
} 

OUTROS = { 
    "danger": ("请紧急避让", "outro_danger.wav"), 
    "warning": ("请注意避让", "outro_warning.wav"), 
    "notice": ("", None)  
} 

SPECIAL_OUTROS = { 
    "red_light": ("请禁止前行", "outro_red_stop.wav"), 
    "green_light": ("请通行", "outro_green_go.wav") 
} 

FORCE_NOTICE_OBJECTS = ["green_light", "blind_road", "crosswalk"] 

OBJECTS = { 
    "person": ("行人", "obj_person.wav"), 
    "car": ("小汽车", "obj_car.wav"), 
    "bus": ("公交车", "obj_bus.wav"), 
    "truck": ("大卡车", "obj_truck.wav"), 

    "dog": ("小狗", "obj_dog.wav"), 
    "motorcycle": ("摩托车", "obj_motorcycle.wav"), 
    "bicycle": ("自行车", "obj_bicycle.wav"), 
    "tricycle": ("三轮车", "obj_tricycle.wav"), 
    "green_light": ("绿灯", "obj_green_light.wav"), 
    "red_light": ("红灯", "obj_red_light.wav"), 
    "sign": ("指示牌", "obj_sign.wav"), 
    "blind_road": ("盲道", "obj_blind_road.wav"), 
    "crosswalk": ("斑马线", "obj_crosswalk.wav"), 
    "warning_column": ("警示柱", "obj_warning_column.wav"), 
    "reflective_cone": ("反光锥", "obj_reflective_cone.wav"), 
    "roadblock": ("路障", "obj_roadblock.wav"), 
    "pole": ("立柱", "obj_pole.wav"), 
    "tree": ("树木", "obj_tree.wav"), 
    "fire_hydrant": ("消防栓", "obj_fire_hydrant.wav"), 
    "ashcan": ("垃圾桶", "obj_ashcan.wav"), 
} 

def float_to_chinese_phrase(val): 
    cn_digits = {0: "零", 1: "一", 2: "二", 3: "三", 4: "四", 5: "五", 6: "六", 7: "七", 8: "八", 9: "九", 10: "十"} 
    str_val = f"{val:.1f}" 
    parts = str_val.split('.') 
    integer_part = int(parts[0]) 
    decimal_part = int(parts[1]) 
    if decimal_part == 0: 
        if integer_part == 2: return "两米有" 
        elif integer_part == 10: return "十米有" 
        else: return f"{cn_digits[integer_part]}米有" 
    else: 
        int_str = "两" if integer_part == 2 else cn_digits[integer_part] 
        dec_str = cn_digits[decimal_part] 
        return f"{int_str}点{dec_str}米有" 

DIST_MAP = {} 
for i in range(5, 51): 
    val = round(i * 0.1, 1) 
    DIST_MAP[f"{val:.1f}"] = float_to_chinese_phrase(val) 
for i in range(11, 21): 
    val = round(i * 0.5, 1) 
    DIST_MAP[f"{val:.1f}"] = float_to_chinese_phrase(val) 

class SegmentedAnnouncer: 
    def __init__(self): 
        print("4. 初始化 SegmentedAnnouncer 类...") 
        self.last_announced = {}  
        self.required_files = [] 

        self.next_play = None         
        self.new_task_event = threading.Event() 
        self.lock = threading.Lock() 
        
        self.play_thread = threading.Thread(target=self._play_worker, daemon=True) 
        
        # 添加开机专属语音的缓存自检项
        self.required_files.append(("startup", "startup.wav"))

        for _, (_, fname) in LEVELS.items(): 
            if fname: self.required_files.append((_, fname)) 
        for _, (_, fname) in DIRECTIONS.items(): 
            self.required_files.append((_, fname)) 
        for _, (_, fname) in OUTROS.items(): 
            if fname: self.required_files.append((_, fname)) 
        for _, (_, fname) in SPECIAL_OUTROS.items(): 
            self.required_files.append((_, fname)) 
        for key, (_, fname) in OBJECTS.items(): 
            self.required_files.append((f"obj_{key}", fname)) 
        for d_key_str, text in DIST_MAP.items(): 
            self.required_files.append((f"dist_{d_key_str}", f"dist_{d_key_str}m.wav")) 
            
        missing = [item for item in self.required_files if not os.path.exists(os.path.join(CACHE_DIR, item[1]))] 
        
        if len(missing) > 0: 
            print(f"5. 检测到缺失 {len(missing)} 个语音组件，准备导入 sherpa_onnx 生成") 
            self._generate_base_segments(missing) 
        else: 
            print("5. 语音组件完整，无需重新生成。") 
            
        self.play_thread.start() 
        print("6. 后台播放线程已启动。") 

    def _generate_base_segments(self, missing_list): 
        try: 
            print("  尝试导入 sherpa_onnx 库") 
            import sherpa_onnx
            print("  sherpa_onnx 导入成功，正在初始化 TTS 模型模型...") 
        except Exception as e: 
            print(f" 导入 sherpa_onnx 库失败") 
            return

        TTS_MODEL_DIR = "./yuyinmodel/vits-zh-aishell3" 
        vits_config = sherpa_onnx.OfflineTtsVitsModelConfig( 
            model=os.path.join(TTS_MODEL_DIR, "vits-aishell3.int8.onnx"), 
            lexicon=os.path.join(TTS_MODEL_DIR, "lexicon.txt"), 
            tokens=os.path.join(TTS_MODEL_DIR, "tokens.txt"), 
            data_dir=TTS_MODEL_DIR, 
        ) 
        model_config = sherpa_onnx.OfflineTtsModelConfig(vits=vits_config, num_threads=3, debug=False) 
        config = sherpa_onnx.OfflineTtsConfig(model=model_config, rule_fars=os.path.join(TTS_MODEL_DIR, "rule.far"), max_num_sentences=1) 
        tts = sherpa_onnx.OfflineTts(config) 
        print("  -> TTS 模型初始化成功。开始生成...") 

        for key_id, filename in missing_list: 
            text = "" 
            
            if filename == "startup.wav":
                text = "系统已启动，开始测试"
            else:
                for k, (t, f) in LEVELS.items(): 
                    if f == filename: text = t
                for k, (t, f) in DIRECTIONS.items(): 
                    if f == filename: text = t
                for k, (t, f) in OUTROS.items(): 
                    if f == filename: text = t
                for k, (t, f) in SPECIAL_OUTROS.items(): 
                    if f == filename: text = t
                for k, (t, f) in OBJECTS.items(): 
                    if f == filename: text = t
                if filename.startswith("dist_") and filename.endswith("m.wav"): 
                    d_key = filename.replace("dist_", "").replace("m.wav", "") 
                    if d_key in DIST_MAP: text = DIST_MAP[d_key] 

            filepath = os.path.join(CACHE_DIR, filename) 
            print(f"     [+] 正在生成：{text} -> {filename}") 
            
            audio = tts.generate(text, sid=10, speed=1.0) 
            raw_samples = np.array(audio.samples) 
            max_val = np.max(np.abs(raw_samples)) 
            
            # 使用高爆饱和限振幅
            norm_samples = raw_samples / max_val * 0.99 if max_val > 0 else raw_samples
            int16_samples = (norm_samples * 32767).astype(np.int16) 
            
            with wave.open(filepath, "wb") as wf: 
                wf.setnchannels(1) 
                wf.setsampwidth(2) 
                wf.setframerate(audio.sample_rate) 
                wf.writeframes(int16_samples.tobytes()) 
        print(" 基础组件补全完毕。") 

    def _join_wavs(self, wav_paths, output_path): 
        target_sr = 22050
        target_ch = 1
        target_width = 2
        combined_samples = [] 
        
        for path in wav_paths: 
            if not os.path.exists(path): continue
            with wave.open(path, 'rb') as w: 
                sr = w.getframerate() 
                ch = w.getnchannels() 
                width = w.getsampwidth() 
                nframes = w.getnframes() 
                raw_data = w.readframes(nframes) 
                
                if width == 2: 
                    samples = np.frombuffer(raw_data, dtype=np.int16).astype(np.float32) / 32768.0
                elif width == 1: 
                    samples = (np.frombuffer(raw_data, dtype=np.uint8).astype(np.float32) - 128.0) / 128.0
                else: 
                    continue
                
                if ch > 1: 
                    samples = samples.reshape(-1, ch).mean(axis=1) 
                
                if sr != target_sr: 
                    duration = len(samples) / sr
                    num_target = int(duration * target_sr) 
                    samples = np.interp(np.linspace(0, len(samples)-1, num_target), np.arange(len(samples)), samples) 
                combined_samples.append(samples) 
                
        if len(combined_samples) > 0: 
            final_data = np.concatenate(combined_samples) 
            
            # 2.5倍响度增益 + 软限幅防爆音
            amplified_data = final_data * 2.5
            limited_data = np.tanh(amplified_data) 
            
            int16_samples = (limited_data * 32767).astype(np.int16) 
            
            with wave.open(output_path, 'wb') as w: 
                w.setnchannels(target_ch) 
                w.setsampwidth(target_width) 
                w.setframerate(target_sr) 
                w.writeframes(int16_samples.tobytes()) 

    # 💥 新增 say 接口，用于播报开机等单段自定义内置语音
    def say(self, text):
        if text == "系统已启动，开始测试":
            filepath = os.path.join(CACHE_DIR, "startup.wav")
            if os.path.exists(filepath):
                with self.lock:
                    self.next_play = ([filepath], text)
                self.new_task_event.set()
            else:
                # 硬件异常时的备份方案
                os.system('espeak -v zh "系统已启动，开始测试" &')

    # 💥 新增 announce 接口，指向 say
    def announce(self, text):
        self.say(text)

    def play_alert(self, label_en, distance_meters, direction="center"): 
        if label_en not in OBJECTS or label_en == "sign": return

        if distance_meters < 0.5: 
            dist_val, dist_key_str = 0.5, "0.5" 
        elif distance_meters > 10.0: 
            return  
        elif distance_meters <= 5.0: 
            dist_val = max(0.5, min(5.0, round(distance_meters, 1))) 
            dist_key_str = f"{dist_val:.1f}" 
        else: 
            dist_val = max(5.0, min(10.0, round(distance_meters * 2) / 2)) 
            dist_key_str = f"{dist_val:.1f}" 

        if label_en in FORCE_NOTICE_OBJECTS: 
            level, cooldown = "notice", 4.0
        else: 
            if dist_val < 2.0: level, cooldown = "danger", 2.0  
            elif dist_val < 5.0: level, cooldown = "warning", 3.5  
            else: level, cooldown = "notice", 5.5  

        now = time.time() 
        track_key = f"{label_en}_{level}_{direction}" 
        if track_key in self.last_announced: 
            if now - self.last_announced[track_key] < cooldown: return  
        self.last_announced[track_key] = now

        parts = [] 
        if LEVELS[level][1]: 
            parts.append(os.path.join(CACHE_DIR, LEVELS[level][1])) 
        if direction in DIRECTIONS: 
            parts.append(os.path.join(CACHE_DIR, DIRECTIONS[direction][1])) 
        parts.append(os.path.join(CACHE_DIR, f"dist_{dist_key_str}m.wav")) 
        parts.append(os.path.join(CACHE_DIR, OBJECTS[label_en][1])) 
        
        outro_text = "" 
        if label_en in SPECIAL_OUTROS: 
            outro_text, outro_fname = SPECIAL_OUTROS[label_en] 
            parts.append(os.path.join(CACHE_DIR, outro_fname)) 
        else: 
            if OUTROS[level][1]: 
                outro_text = OUTROS[level][0] 
                parts.append(os.path.join(CACHE_DIR, OUTROS[level][1])) 

        dir_text = DIRECTIONS[direction][0] if direction in DIRECTIONS else "" 
        lvl_text = LEVELS[level][0] 
        zh_desc = f"{lvl_text}{dir_text}{DIST_MAP[dist_key_str]}{OBJECTS[label_en][0]}{outro_text}" 
        
        # 这样如果在播放期间来了10个新检测，它们只有最后一个（最新的）能存活，其余旧任务当即被覆盖抛弃。 
        with self.lock: 
            self.next_play = (parts, zh_desc) 
        self.new_task_event.set() 

    def _play_worker(self): 
        while True: 
            # 阻塞等待新任务到达信号
            self.new_task_event.wait() 
            
            # 使用锁安全地取出当前最新任务，同时清空槽，重置信号状态
            with self.lock: 
                task = self.next_play
                self.next_play = None
                self.new_task_event.clear() 
                
            if task is None: 
                continue
                
            parts, zh_desc = task
            self._join_wavs(parts, TEMP_PLAY_WAV) 
            print(f"[正在播放音频]  「{zh_desc}」") 
            
            cmd = ['aplay'] 
            if ALSA_DEVICE: 
                cmd.extend(['-D', ALSA_DEVICE]) 
            cmd.append(TEMP_PLAY_WAV) 

            # 进行硬件播放（直到播毕） 
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) 
            
            # 播放完后，短歇 0.05 秒（给人耳一个极短的气息停顿，避免多个词黏在一块分不开） 
            time.sleep(0.05) 

if __name__ == "__main__": 
    print("3. 进入主执行入口 (main)...") 
    announcer = SegmentedAnnouncer() 
    print(" 开始播放测试音频...") 
    
    # 💥 测试开机播报语音
    announcer.say("系统已启动，开始测试")
    time.sleep(3.0) 
    
    # 模拟连续快速传入 3 个目标： 
    # 无论中间发了什么，前一个播毕后，下一次必然播最新采集的那一个
    announcer.play_alert("person", 1.34, "left")    # 瞬间被触发播放
    time.sleep(0.1) 
    announcer.play_alert("car", 2.5, "right")        #（会被覆盖弃用） 
    time.sleep(0.1) 
    announcer.play_alert("dog", 4.1, "center")       #（最新数据：将在前一个播毕后立刻开始播放它） 
    
    time.sleep(6.0) 
    print("测试完毕。")
