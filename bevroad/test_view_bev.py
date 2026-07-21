import os
import tempfile
import unittest

from bevroad.view_bev import load_config, save_config


class TestViewBev(unittest.TestCase):
    def setUp(self):
        self.test_dir = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.test_dir.cleanup()

    def test_load_config_non_existent(self):
        non_existent_path = os.path.join(self.test_dir.name, 'does_not_exist.json')
        config = load_config(non_existent_path)
        self.assertEqual(config, {})

    def test_save_and_load_config(self):
        config_path = os.path.join(self.test_dir.name, 'test_config.json')
        test_data = {
            'top_y': 100,
            'bottom_y': 200,
            'top_width': 300,
            'bottom_width': 400,
            'offset_x': 50,
            'show_grid': 1,
        }

        save_config(config_path, test_data)

        # Verify file was written
        self.assertTrue(os.path.exists(config_path))

        # Load and verify content
        loaded_data = load_config(config_path)
        self.assertEqual(loaded_data, test_data)

    def test_load_config_invalid_json(self):
        invalid_path = os.path.join(self.test_dir.name, 'invalid.json')
        with open(invalid_path, 'w') as f:
            f.write('{invalid-json:')

        config = load_config(invalid_path)
        self.assertEqual(config, {})


if __name__ == '__main__':
    unittest.main()
