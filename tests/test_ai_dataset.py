import unittest
import numpy as np

from ai.generate_dataset import (
    ACTION_NAMES,
    expert_action,
    generate_dataset,
    normalize_features,
)


class AiDatasetTests(unittest.TestCase):
    def test_teacher_labels_known_scenarios(self):
        self.assertEqual(expert_action(1500, 1000, 900, 0.0, 0.0), 0)
        self.assertEqual(expert_action(200, 1200, 250, 0.0, 0.0), 1)
        self.assertEqual(expert_action(200, 250, 1300, 0.0, 0.0), 2)
        self.assertEqual(expert_action(150, 200, 250, 0.0, 0.0), 3)
        self.assertEqual(expert_action(420, 1000, 1200, 0.3, 0.0), 4)

    def test_generator_is_deterministic_and_well_formed(self):
        x1, y1 = generate_dataset(500, seed=19)
        x2, y2 = generate_dataset(500, seed=19)
        self.assertTrue(np.array_equal(x1, x2))
        self.assertTrue(np.array_equal(y1, y2))
        self.assertEqual(x1.shape, (500, 5))
        self.assertTrue(set(np.unique(y1)).issubset(set(range(len(ACTION_NAMES)))))

    def test_feature_normalization_bounds(self):
        raw = np.array([[3000, -10, 1000, 4.0, -360.0]], dtype=np.float32)
        normalized = normalize_features(raw)
        self.assertTrue(np.allclose(normalized, [[1.0, 0.0, 0.5, 1.0, -1.0]]))


if __name__ == "__main__":
    unittest.main()
