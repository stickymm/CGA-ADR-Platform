import cv2
import json
import socket
import time
from pathlib import Path

import numpy as np
import yaml

from .gstreamer_class import CameraStream
# ==========================================
# --- CONFIGURATION TOGGLES ---
# ==========================================
USE_VIDEO_FILE = False  # True: Use MKV video file | False: Use Live Camera
PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSETS_DIR = PROJECT_ROOT / "assets"
VIDEO_FILE_PATH = PROJECT_ROOT / "F3video1_Flipped.mkv"
FLIP_CAMERA = True      # Set to True if the physical camera is mounted upside down
# ==========================================

class HailoYOLO:
    """Custom Hardware Wrapper for the Hailo-10H AI Hat using HailoRT 5.x"""
    def __init__(self, hef_path):
        from hailo_platform import VDevice, FormatType

        self.vdevice = VDevice()
        self.infer_model = self.vdevice.create_infer_model(hef_path)
        self.infer_model.set_batch_size(1)

        for input_stream in self.infer_model.inputs:
            input_stream.set_format_type(FormatType.UINT8)
        for output_stream in self.infer_model.outputs:
            output_stream.set_format_type(FormatType.FLOAT32)

        self.configured_infer_model = self.infer_model.configure()
        self.bindings = self.configured_infer_model.create_bindings()

        self.out_buffers = {}
        for output_stream in self.infer_model.outputs:
            out_shape = output_stream.shape
            buffer = np.empty(out_shape, dtype=np.float32)
            self.out_buffers[output_stream.name] = buffer
            self.bindings.output(output_stream.name).set_buffer(buffer)

        # Pre-compute YOLOv8 Anchor Grid for DFL decoding
        def _make_grid(nx, ny):
            yv, xv = np.meshgrid(np.arange(ny), np.arange(nx), indexing='ij')
            return np.stack((xv, yv), axis=2).reshape(-1, 2) + 0.5

        strides = [8, 16, 32]
        grid, stride_tensor = [],[]
        for s in strides:
            nx, ny = 640 // s, 640 // s
            grid.append(_make_grid(nx, ny))
            stride_tensor.append(np.full((nx * ny, 1), s))

        self.anchors = np.concatenate(grid, axis=0) # Shape: (8400, 2)
        self.strides = np.concatenate(stride_tensor, axis=0) # Shape: (8400, 1)
        self.dfl_weights = np.arange(16, dtype=np.float32) # Weights for Softmax

    def predict(self, frame, conf=0.5):
        h, w = frame.shape[:2]

        # INTER_NEAREST is much faster on CPU than default linear interpolation
        img = cv2.resize(frame, (640, 640), interpolation=cv2.INTER_NEAREST)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.expand_dims(img, axis=0)

        input_name = self.infer_model.inputs[0].name
        self.bindings.input(input_name).set_buffer(img)
        self.configured_infer_model.run([self.bindings], 1000)

        buf_values = list(self.out_buffers.values())
        b1, b2 = np.squeeze(buf_values[0]), np.squeeze(buf_values[1])

        # 1. ROBUST IDENTIFICATION
        if b1.size > b2.size:
            dfl_buf, scores_buf = b1, b2
        else:
            dfl_buf, scores_buf = b2, b1

        # 2. ROBUST EXTRACTION (Bypass Hailo's 16-channel padding bug)
        dfl_buf = dfl_buf.reshape(8400, 64)
        scores = scores_buf.reshape(8400, -1)[:, 0]

        # 3. FILTER FIRST
        mask = scores >= conf
        filtered_scores = scores[mask]
        filtered_dfl = dfl_buf[mask]
        filtered_anchors = self.anchors[mask]
        filtered_strides = self.strides[mask]

        boxes = []
        if len(filtered_scores) > 0:
            # 4. DECODE DFL ONLY ON CONFIDENT BOXES
            dfl_reshaped = filtered_dfl.reshape(-1, 4, 16)

            exp_dfl = np.exp(dfl_reshaped - np.max(dfl_reshaped, axis=2, keepdims=True))
            softmax_dfl = exp_dfl / np.sum(exp_dfl, axis=2, keepdims=True)
            dfl_out = np.sum(softmax_dfl * self.dfl_weights, axis=2)

            lt = dfl_out[:, :2]
            rb = dfl_out[:, 2:]
            x1y1 = filtered_anchors - lt
            x2y2 = filtered_anchors + rb

            pred_boxes = np.concatenate((x1y1, x2y2), axis=1) * filtered_strides

            # Fast vectorized scaling to original resolution
            scale_array = np.array([w / 640.0, h / 640.0, w / 640.0, h / 640.0], dtype=np.float32)
            pred_boxes *= scale_array

            bboxes_for_nms = pred_boxes.tolist()
            scores_for_nms = filtered_scores.tolist()

            # 5. NMS Cleanup
            keep_indices = cv2.dnn.NMSBoxes(bboxes_for_nms, scores_for_nms, conf, 0.4)

            if len(keep_indices) > 0:
                for i in keep_indices.flatten():
                    bx1, by1, bx2, by2 = bboxes_for_nms[i]
                    boxes.append([int(bx1), int(by1), int(bx2), int(by2)])

        return boxes


class OpenCVProcessing():
    def __init__(self, cam: CameraStream):
        self.cam = cam

        self.MODEL_PATH = ASSETS_DIR / "best.hef"

        if USE_VIDEO_FILE:
            self.CALIB_PATH = ASSETS_DIR / "tevs_CAM1_fisheye_calibration.yaml"
        else:
            self.CALIB_PATH = ASSETS_DIR / "tevs_CAM2_fisheye_calibration.yaml"

        self.HALF_SIZE = 0.97155 / 2.0
        self.GATE_3D_CORNERS = np.array([
            [-self.HALF_SIZE, -self.HALF_SIZE, 0],
            [ self.HALF_SIZE, -self.HALF_SIZE, 0],
            [ self.HALF_SIZE,  self.HALF_SIZE, 0],
            [-self.HALF_SIZE,  self.HALF_SIZE, 0]
        ], dtype=np.float32)

        self.LOWER_ORANGE = np.array([0, 30, 20])
        self.UPPER_ORANGE = np.array([30, 255, 255])

        print(f"[INFO] Loading Camera Calibration: {self.CALIB_PATH}")
        with open(self.CALIB_PATH, "r", encoding="utf-8") as f:
            calib = yaml.safe_load(f)
        self.K = np.array(calib['camera_matrix'], dtype=np.float64)
        self.D = np.array(calib['dist_coeffs'], dtype=np.float64)

        print("[INFO] Loading Hailo-10H Hardware Model...")
        self.model = HailoYOLO(str(self.MODEL_PATH))

        # --- SETUP UDP TELEMETRY PUBLISHER ---
        self.udp_ip = "127.0.0.1"  # Localhost
        self.udp_port = 5050       # Arbitrary open port
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"[INFO] Broadcasting telemetry to {self.udp_ip}:{self.udp_port}")

    def find_math_corners(self, crop_img):
        h, w = crop_img.shape[:2]
        yolo_corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)

        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.LOWER_ORANGE, self.UPPER_ORANGE)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours or cv2.contourArea(max(contours, key=cv2.contourArea)) < 500:
            return yolo_corners, mask, [0, 1, 2, 3]

        largest_contour = max(contours, key=cv2.contourArea)
        pts = largest_contour.reshape(-1, 2)

        tl = pts[np.argmin(np.linalg.norm(pts - yolo_corners[0], axis=1))].astype(np.float32)
        tr = pts[np.argmin(np.linalg.norm(pts - yolo_corners[1], axis=1))].astype(np.float32)
        br = pts[np.argmin(np.linalg.norm(pts - yolo_corners[2], axis=1))].astype(np.float32)
        bl = pts[np.argmin(np.linalg.norm(pts - yolo_corners[3], axis=1))].astype(np.float32)

        corners = [tl, tr, br, bl]
        threshold = 0.20 * max(w, h)

        bad_indices = []
        for i in range(4):
            if np.linalg.norm(corners[i] - yolo_corners[i]) > threshold:
                bad_indices.append(i)

        if len(bad_indices) == 1:
            bad_idx = bad_indices[0]
            if bad_idx == 0: corners[0] = corners[1] + (corners[3] - corners[2])
            elif bad_idx == 1: corners[1] = corners[0] + (corners[2] - corners[3])
            elif bad_idx == 2: corners[2] = corners[3] + (corners[1] - corners[0])
            elif bad_idx == 3: corners[3] = corners[0] + (corners[2] - corners[1])
        elif len(bad_indices) > 1:
            for idx in bad_indices:
                corners[idx] = yolo_corners[idx]

        return np.array(corners, dtype=np.float32), mask, bad_indices

    def cv_processing_loop(self):
        prev_tracked_gates = []
        ALPHA = 0.3

        print("[INFO] OpenCV Processing Loop Started.")

        if USE_VIDEO_FILE:
            print(f"[INFO] Using Video File Mode: {VIDEO_FILE_PATH}")
            cap = cv2.VideoCapture(str(VIDEO_FILE_PATH))
        else:
            print("[INFO] Using Live Camera Mode")

        while True:
            if not USE_VIDEO_FILE and not self.cam.is_camera_running():
                break

            start_time = time.time()
            if USE_VIDEO_FILE:
                ret, frame = cap.read()
                if not ret:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
            else:
                frame = self.cam.get_latest_cv_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

            # --- FLIP FRAME IF MOUNTED UPSIDE DOWN ---
            if FLIP_CAMERA:
                frame = cv2.flip(frame, -1)  # '-1' flips both X and Y axes (180-deg rotation)

            height, width = frame.shape[:2]
            global_mask = np.zeros((height, width), dtype=np.uint8)

            boxes = self.model.predict(frame, conf=0.5)
            boxes = boxes[:3]

            current_tracked_gates = []

            for box in boxes:
                x1, y1, x2, y2 = box
                is_partial = (x1 <= 5 or y1 <= 5 or x2 >= width - 5 or y2 >= height - 5)
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(width, x2), min(height, y2)

                box_color = (0, 0, 255) if is_partial else (255, 0, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)

                crop = frame[y1:y2, x1:x2]
                if crop.size > 0:
                    crop_corners, crop_mask, bad_indices = self.find_math_corners(crop)
                    global_mask[y1:y2, x1:x2] = crop_mask

                    if crop_corners is not None:
                        full_img_corners = crop_corners + np.array([x1, y1])
                        colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]

                        for i, corner in enumerate(full_img_corners):
                            color = (255, 0, 255) if i in bad_indices else colors[i]
                            cv2.circle(frame, (int(corner[0]), int(corner[1])), 6, color, -1)

                        if not is_partial:
                            undistorted_corners = cv2.fisheye.undistortPoints(
                                full_img_corners.reshape(-1, 1, 2), self.K, self.D, P=self.K
                            )
                            
                            pts2d = undistorted_corners.reshape(-1,2)
                            
                            ## validate corner geometry before calling solvePnP
                            
                            # Reject NaN / inf
                            if not np.isfinite(pts2d).all():
                                print("[WARN] Invalid gate corners:", pts2d)
                                continue
                            
                            # Reject udplicate / nearly-duplicate corners
                            min_corner_dist = min(
                                np.linalg.norm(pts2d[i] - pts2d[j])
                                for i in range(4)
                                for j in range(i + 1, 4)
                            )
                            
                            if min_corner_dist < 3.0:
                                print("[WARN] Gate corners collapsed:", pts2d)
                                continue
                            
                                                        
                            # Reject tiny / degenerate quadrilaterals
                            gate_area = abs(
                                cv2.contourArea(
                                    pts2d.astype(np.float32)
                                )
                            )
                            
                            if gate_area < 25.0:
                                print("[WARN] Gate area too small:", gate_area)
                                continue

                            # Exactly reproduce the coordinate-space SQPnP cares about.
                            normalized = cv2.undistortPoints(
                                pts2d.reshape(-1, 1, 2),
                                self.K,
                                None
                            ).reshape(-1, 2)

                            point_variance = (
                                np.var(normalized[:, 0]) +
                                np.var(normalized[:, 1])
                            )

                            if point_variance < 1e-5:
                                print(
                                    "[WARN] SQPnP point variance too small:",
                                    point_variance,
                                    pts2d
                                )
                                continue
                            
                            try: 
                                success, rvec, tvec = cv2.solvePnP(
                                    self.GATE_3D_CORNERS, undistorted_corners, self.K, np.zeros(4), flags=cv2.SOLVEPNP_SQPNP
                                )
                                
                            except cv2.error as e:
                                print("[WARN] solvePnP failed:", e)
                                continue


                            if success:
                                rvec, tvec = cv2.solvePnPRefineLM(
                                    self.GATE_3D_CORNERS, undistorted_corners, self.K, np.zeros(4), rvec, tvec
                                )

                                cv_x, cv_y, cv_z = tvec[0][0], tvec[1][0], tvec[2][0]
                                distance = np.linalg.norm(tvec)

                                if cv_z < 0 or distance > 30.0:
                                    continue

                                best_match = None
                                min_dist = float('inf')
                                for ptg in prev_tracked_gates:
                                    jump_dist = np.hypot(cv_x - ptg['cv_x'], cv_z - ptg['cv_z'])
                                    if jump_dist < 1.5 and jump_dist < min_dist:
                                        min_dist = jump_dist
                                        best_match = ptg

                                if best_match is not None:
                                    smooth_tvec = (ALPHA * tvec) + ((1.0 - ALPHA) * best_match['tvec'])
                                    smooth_rvec = (ALPHA * rvec) + ((1.0 - ALPHA) * best_match['rvec'])
                                else:
                                    smooth_tvec = tvec
                                    smooth_rvec = rvec

                                drone_fwd_x = smooth_tvec[2][0]
                                drone_rgt_y = smooth_tvec[0][0]
                                drone_dwn_z = smooth_tvec[1][0]

                                rmat, _ = cv2.Rodrigues(smooth_rvec)
                                proj_matrix = np.hstack((rmat, smooth_tvec))
                                _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(proj_matrix)
                                pitch, yaw, roll = euler_angles.flatten()

                                current_tracked_gates.append({
                                    'tvec': smooth_tvec, 'rvec': smooth_rvec,
                                    'cv_x': cv_x, 'cv_z': cv_z, 'dist': distance,
                                    'd_x': drone_fwd_x, 'd_y': drone_rgt_y, 'd_z': drone_dwn_z,
                                    'roll': roll, 'pitch': pitch, 'yaw': yaw
                                })

                                cv2.drawFrameAxes(frame, self.K, np.zeros(4), smooth_rvec, smooth_tvec, 0.5, 3)
                                cv2.putText(frame, f"Dist: {distance:.2f}m", (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            prev_tracked_gates = current_tracked_gates

            # --- BUILD TELEMETRY MATRIX ---
            current_tracked_gates.sort(key=lambda g: g['dist'])

            telemetry_matrix = []
            for g in current_tracked_gates[:3]:
                row = [
                    round(g['dist'], 3),
                    round(g['d_x'], 3), round(g['d_y'], 3), round(g['d_z'], 3),
                    round(g['roll'], 2), round(g['pitch'], 2), round(g['yaw'], 2)
                ]
                telemetry_matrix.append(row)

            while len(telemetry_matrix) < 3:
                telemetry_matrix.append([999.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0])

            # --- VISUALIZE TELEMETRY MATRIX ON SCREEN ---
            y_offset = 70
            cv2.putText(frame, "Telemetry Matrix [Dist, Fwd(X), Rgt(Y), Dwn(Z), Roll, Pitch, Yaw]:",
                        (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            for i, row in enumerate(telemetry_matrix):
                # Format each number to have exactly 2 decimal places and padding so they line up cleanly
                row_str = " | ".join([f"{val:6.2f}" for val in row])
                text_color = (0, 255, 0) if row[0] != 999.0 else (0, 0, 255) # Green if real, Red if 999

                cv2.putText(frame, f"G{i+1}: [{row_str}]",
                            (10, y_offset + 25 + (i * 25)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
            # --------------------------------

            # Calculate and Draw FPS
            # Calculate and Draw FPS
            process_time = time.time() - start_time
            fps = 1.0 / process_time if process_time > 0 else 0
            cv2.putText(frame, f"Pipeline FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            # =========================================================
            # --- BROADCAST UDP TELEMETRY ---
            # =========================================================
            telemetry_payload = {
                "fps": round(fps, 1),
                "gates": telemetry_matrix
            }
            try:
                # Convert dictionary to JSON string, then encode to bytes and send
                message = json.dumps(telemetry_payload).encode('utf-8')
                self.udp_sock.sendto(message, (self.udp_ip, self.udp_port))
            except Exception as e:
                pass # If network is busy, just drop the frame and move on
            # =========================================================

            # Fast Encode for Web Stream
            stream_frame = cv2.resize(frame, (960, 540), interpolation=cv2.INTER_LINEAR)

            # Fast Encode for Web Stream

            ok, buffer = cv2.imencode(".jpg", stream_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                self.cam.set_latest_processed_jpg(buffer.tobytes())
