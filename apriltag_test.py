import unittest
import cv2
from time import sleep

from pid import PID
from apriltag_processing import *


class AprilTagtest(unittest.TestCase):
    def test_get_frames(self):
        frames = get_frames("AprilTagTest.mkv")

        self.assertEqual(len(frames), 862)

    def _get_tags(self):
        img = cv2.imread("test_image.png")

        at_detector = Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        tags = detect_april_tags(img, at_detector)

        return tags

    def test_detect_april_tags(self):
        tags = self._get_tags()

        self.assertEqual(len(tags), 5)

    def test_determine_errors(self):
        tags = self._get_tags()

        img = cv2.imread("test_image.png")
        width, height, channels = img.shape

        x_error, y_error = determine_error(tags, (width, height))

        print(x_error, y_error)

        self.assertTrue(x_error, 0.18890050607214068)
        self.assertAlmostEqual(y_error, -0.1942930989306754)

    def test_calculate_pid_output(self):
        x_error, y_error = (0.2, 0.3)
        horizontal_pid, vertical_pid = PID(
            K_p=0.1, K_i=0.0, K_d=0.01, integral_limit=1
        ), PID(K_p=0.1, K_i=0.0, K_d=0.01, integral_limit=1)

        sleep(1.0)

        x_output, y_output = calculate_pid_output(
            (x_error, y_error), (horizontal_pid, vertical_pid)
        )

        # self.assertAlmostEqual(x_output, 0.022)
        # self.assertAlmostEqual(y_output, 0.033)

        self.assertTrue(np.isclose(x_output, 0.022, atol=1e-4))
        self.assertTrue(np.isclose(y_output, 0.033, atol=1e-4))


if __name__ == "__main__":
    unittest.main()
