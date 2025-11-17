"""
Pose Detection Module
Uses MediaPipe for human pose detection and tracking
"""

import cv2
import mediapipe as mp
import numpy as np
from typing import Optional, Tuple, List, Dict
from dataclasses import dataclass
from collections import deque
import threading
import time

@dataclass
class Person:
    """Represents a detected person with pose information"""
    id: int
    landmarks: np.ndarray
    visibility: np.ndarray
    bbox: Tuple[int, int, int, int]  # x, y, width, height
    center: Tuple[float, float]  # normalized coordinates
    confidence: float
    timestamp: float

class PoseDetector:
    """MediaPipe-based pose detection with tracking"""
    
    # MediaPipe pose landmark indices
    POSE_LANDMARKS = {
        'nose': 0,
        'left_eye_inner': 1,
        'left_eye': 2,
        'left_eye_outer': 3,
        'right_eye_inner': 4,
        'right_eye': 5,
        'right_eye_outer': 6,
        'left_ear': 7,
        'right_ear': 8,
        'mouth_left': 9,
        'mouth_right': 10,
        'left_shoulder': 11,
        'right_shoulder': 12,
        'left_elbow': 13,
        'right_elbow': 14,
        'left_wrist': 15,
        'right_wrist': 16,
        'left_pinky': 17,
        'right_pinky': 18,
        'left_index': 19,
        'right_index': 20,
        'left_thumb': 21,
        'right_thumb': 22,
        'left_hip': 23,
        'right_hip': 24,
        'left_knee': 25,
        'right_knee': 26,
        'left_ankle': 27,
        'right_ankle': 28,
        'left_heel': 29,
        'right_heel': 30,
        'left_foot_index': 31,
        'right_foot_index': 32
    }
    
    def __init__(self, config: dict):
        self.config = config
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Initialize pose model
        self.pose = self.mp_pose.Pose(
            model_complexity=config.get('model_complexity', 1),
            min_detection_confidence=config.get('min_detection_confidence', 0.5),
            min_tracking_confidence=config.get('min_tracking_confidence', 0.5),
            enable_segmentation=config.get('enable_segmentation', False),
            smooth_landmarks=config.get('smooth_landmarks', True)
        )
        
        # Tracking variables
        self.persons = {}
        self.next_person_id = 0
        self.tracking_history = deque(maxlen=30)
        
        # Performance metrics
        self.processing_time = 0
        self.detection_fps = 0
        self.last_fps_time = time.time()
        self.frame_count = 0
        
        # Thread safety
        self.lock = threading.Lock()
    
    def process_frame(self, frame: np.ndarray) -> Tuple[List[Person], np.ndarray]:
        """Process a frame and detect poses"""
        start_time = time.time()
        
        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the frame
        results = self.pose.process(rgb_frame)
        
        persons = []
        annotated_frame = frame.copy()
        
        if results.pose_landmarks:
            # Extract person information
            person = self._extract_person(results, frame.shape)
            persons.append(person)
            
            # Draw pose landmarks
            if self.config.get('show_skeleton', True):
                self.mp_drawing.draw_landmarks(
                    annotated_frame,
                    results.pose_landmarks,
                    self.mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
                )
            
            # Draw bounding box and info
            self._draw_person_info(annotated_frame, person)
        
        # Update performance metrics
        self.processing_time = time.time() - start_time
        self._update_fps()
        
        return persons, annotated_frame
    
    def get_target_position(self, persons: List[Person], 
                          target_part: str = 'nose',
                          mode: str = 'center') -> Optional[Tuple[float, float]]:
        """Get target position for tracking based on detection mode"""
        if not persons:
            return None
        
        if mode == 'center':
            # Use the person closest to center
            frame_center = 0.5
            closest_person = min(persons, 
                               key=lambda p: abs(p.center[0] - frame_center))
            target_person = closest_person
        
        elif mode == 'largest':
            # Use the person with largest bounding box
            target_person = max(persons, 
                              key=lambda p: p.bbox[2] * p.bbox[3])
        
        elif mode == 'closest':
            # Use the person with highest confidence
            target_person = max(persons, key=lambda p: p.confidence)
        
        else:
            target_person = persons[0]
        
        # Get specific body part position
        if target_part in self.POSE_LANDMARKS:
            landmark_idx = self.POSE_LANDMARKS[target_part]
            landmark = target_person.landmarks[landmark_idx]
            visibility = target_person.visibility[landmark_idx]
            
            if visibility > self.config.get('min_visibility', 0.5):
                return (landmark[0], landmark[1])
        
        # Fall back to person center
        return target_person.center
    
    def _extract_person(self, results, frame_shape) -> Person:
        """Extract person information from MediaPipe results"""
        height, width, _ = frame_shape
        
        # Extract landmarks
        landmarks = []
        visibility = []
        
        for landmark in results.pose_landmarks.landmark:
            landmarks.append([landmark.x, landmark.y, landmark.z])
            visibility.append(landmark.visibility)
        
        landmarks = np.array(landmarks)
        visibility = np.array(visibility)
        
        # Calculate bounding box
        visible_landmarks = landmarks[visibility > 0.5]
        if len(visible_landmarks) > 0:
            x_coords = visible_landmarks[:, 0] * width
            y_coords = visible_landmarks[:, 1] * height
            
            x_min = int(np.min(x_coords))
            y_min = int(np.min(y_coords))
            x_max = int(np.max(x_coords))
            y_max = int(np.max(y_coords))
            
            bbox = (x_min, y_min, x_max - x_min, y_max - y_min)
            center = ((x_min + x_max) / 2 / width, (y_min + y_max) / 2 / height)
        else:
            bbox = (0, 0, width, height)
            center = (0.5, 0.5)
        
        # Calculate overall confidence
        confidence = np.mean(visibility)
        
        # Create person object
        person = Person(
            id=self.next_person_id,
            landmarks=landmarks,
            visibility=visibility,
            bbox=bbox,
            center=center,
            confidence=confidence,
            timestamp=time.time()
        )
        
        self.next_person_id += 1
        return person
    
    def _draw_person_info(self, frame: np.ndarray, person: Person):
        """Draw bounding box and information for a person"""
        x, y, w, h = person.bbox
        
        # Draw bounding box
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Draw person info
        info_text = f"ID: {person.id} | Conf: {person.confidence:.2f}"
        cv2.putText(frame, info_text, (x, y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw center point
        center_x = int(person.center[0] * frame.shape[1])
        center_y = int(person.center[1] * frame.shape[0])
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
    
    def _update_fps(self):
        """Update detection FPS"""
        self.frame_count += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 1.0:
            self.detection_fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def get_stats(self) -> Dict:
        """Get detection statistics"""
        return {
            'processing_time': self.processing_time * 1000,  # ms
            'detection_fps': self.detection_fps,
            'num_persons': len(self.persons),
            'tracking_history_size': len(self.tracking_history)
        }
    
    def cleanup(self):
        """Clean up resources"""
        if self.pose:
            self.pose.close()


class PositionSmoother:
    """Smooth position tracking to reduce jitter"""
    
    def __init__(self, window_size: int = 5, alpha: float = 0.3):
        self.window_size = window_size
        self.alpha = alpha  # For exponential smoothing
        self.position_history = deque(maxlen=window_size)
        self.smoothed_position = None
    
    def update(self, position: Optional[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        """Update with new position and return smoothed position"""
        if position is None:
            return tuple(self.smoothed_position) if self.smoothed_position is not None else None

        # Ensure numeric 2-tuple
        try:
            px = float(position[0])
            py = float(position[1])
        except Exception:
            # Invalid input, ignore this update
            return tuple(self.smoothed_position) if self.smoothed_position is not None else None

        self.position_history.append((px, py))
        
        if len(self.position_history) < 2:
            self.smoothed_position = position
            return position
        
        # Moving average smoothing
        positions = np.array(self.position_history, dtype=float)
        avg_position = np.mean(positions, axis=0)
        
        # Exponential smoothing
        if self.smoothed_position is None:
            # Store as numpy array for safe arithmetic
            self.smoothed_position = np.asarray(avg_position, dtype=float)
        else:
            # Ensure internal type is numpy array
            try:
                current = np.asarray(self.smoothed_position, dtype=float)
                self.smoothed_position = (
                    float(self.alpha) * avg_position + 
                    (1.0 - float(self.alpha)) * current
                )
            except Exception:
                # Fallback if anything goes wrong
                self.smoothed_position = np.asarray(avg_position, dtype=float)
        
        return (float(self.smoothed_position[0]), float(self.smoothed_position[1]))
    
    def reset(self):
        """Reset the smoother"""
        self.position_history.clear()
        self.smoothed_position = None
