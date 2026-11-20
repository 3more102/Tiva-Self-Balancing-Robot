import unittest

import numpy as np

from ai.train_model import classification_metrics


class TinyMLClassificationMetricsTests(unittest.TestCase):
    def test_confusion_matrix_and_per_class_metrics(self):
        y_true = np.array([0, 0, 1, 1, 2, 2], dtype=np.int64)
        y_pred = np.array([0, 1, 1, 2, 2, 0], dtype=np.int64)

        metrics = classification_metrics(y_true, y_pred, ("A", "B", "C"))

        self.assertEqual(
            metrics["confusion_matrix"],
            [[1, 1, 0], [0, 1, 1], [1, 0, 1]],
        )
        self.assertEqual(metrics["confusion_matrix_labels"], ["A", "B", "C"])
        for key in ("per_class_precision", "per_class_recall", "per_class_f1"):
            for value in metrics[key].values():
                self.assertAlmostEqual(value, 0.5)
        self.assertEqual(metrics["per_class_support"], {"A": 2, "B": 2, "C": 2})
        self.assertAlmostEqual(metrics["macro_precision"], 0.5)
        self.assertAlmostEqual(metrics["macro_recall"], 0.5)
        self.assertAlmostEqual(metrics["macro_f1"], 0.5)
        self.assertAlmostEqual(metrics["weighted_f1"], 0.5)

    def test_missing_class_is_reported_with_zero_metrics(self):
        metrics = classification_metrics(
            np.array([0, 0], dtype=np.int64),
            np.array([0, 1], dtype=np.int64),
            ("A", "B", "C"),
        )

        self.assertEqual(metrics["per_class_support"], {"A": 2, "B": 0, "C": 0})
        self.assertEqual(metrics["per_class_recall"]["B"], 0.0)
        self.assertEqual(metrics["per_class_f1"]["C"], 0.0)
        self.assertEqual(metrics["confusion_matrix"][2], [0, 0, 0])

    def test_rejects_invalid_label_inputs(self):
        valid = np.array([0, 1], dtype=np.int64)
        with self.assertRaises(ValueError):
            classification_metrics(valid.reshape(1, 2), valid)
        with self.assertRaises(ValueError):
            classification_metrics(valid, np.array([0], dtype=np.int64))
        with self.assertRaises(ValueError):
            classification_metrics(np.array([], dtype=np.int64),
                                   np.array([], dtype=np.int64))
        with self.assertRaises(ValueError):
            classification_metrics(np.array([0.0, 1.0]), valid)
        with self.assertRaises(ValueError):
            classification_metrics(valid, np.array([0, 3], dtype=np.int64), ("A", "B"))
        with self.assertRaises(ValueError):
            classification_metrics(valid, valid, ("duplicate", "duplicate"))


if __name__ == "__main__":
    unittest.main()
