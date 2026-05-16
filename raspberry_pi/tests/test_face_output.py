#!/usr/bin/env python3
"""
Face Detection Module Test Script
Real-time output of face detection data with FPS monitoring for testing purposes.
"""

import time
import sys
import os

# Add parent directory to path to import vision module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.face_landmarker import _run_face_opencv
import argparse


class FaceTester:
    """Test class for face detection module with FPS monitoring."""

    def __init__(self):
        self.frame_count = 0
        self.start_time = time.time()
        self.last_fps_time = self.start_time
        self.total_frames = 0

    def detection_callback(self, face_info):
        """
        Callback function called when face is detected.

        Args:
            face_info: dict with keys 'target_x', 'target_y', 'width'
                      Format: {"target_x": int, "target_y": int, "width": int}
        """
        self.frame_count += 1
        self.total_frames += 1

        # Calculate FPS every second
        current_time = time.time()
        elapsed = current_time - self.last_fps_time

        if elapsed >= 1.0:
            fps = self.frame_count / elapsed
            print(f"\n[FPS: {fps:.1f}] Frames in last second: {self.frame_count}")
            self.frame_count = 0
            self.last_fps_time = current_time

        # Print detection data (clean format for testing)
        print(
            f"[DETECTED] X: {face_info['target_x']:4d}, "
            f"Y: {face_info['target_y']:4d}, "
            f"Width: {face_info['width']:3d}px"
        )

    def get_stats(self):
        """Get test statistics."""
        total_time = time.time() - self.start_time
        avg_fps = self.total_frames / total_time if total_time > 0 else 0
        return {
            "total_frames": self.total_frames,
            "total_time": total_time,
            "avg_fps": avg_fps,
        }


def main():
    """Main test function."""
    print("=" * 60)
    print("Face Detection Module Test - Face Detection Output")
    print("=" * 60)
    print("Real-time face detection data with FPS monitoring")
    print("Output format: {target_x: int, target_y: int, width: int}")
    print("Press 'q' in video window to exit")
    print("Press Ctrl+C to force exit")
    print("=" * 60)

    tester = FaceTester()

    try:
        # Run face landmarker with callback
        run_face_landmarker_on_camera(
            camera_id=0, on_detected=tester.detection_callback
        )
    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Test stopped by user")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback

        traceback.print_exc()
    finally:
        # Print final statistics
        stats = tester.get_stats()
        print("\n" + "=" * 60)
        print("Test Statistics:")
        print(f"  Total frames processed: {stats['total_frames']}")
        print(f"  Total time: {stats['total_time']:.2f}s")
        print(f"  Average FPS: {stats['avg_fps']:.1f}")
        print("=" * 60)
        print("Test completed")


if __name__ == "__main__":
    main()
