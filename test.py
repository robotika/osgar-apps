#!/usr/bin/env python

if __name__ == "__main__":
    import logging
    logging.root.level = logging.CRITICAL
    import unittest
    from pathlib import Path
    import os
    import subprocess
    root_dir = Path(__file__).parent
    for project_name in os.listdir(root_dir):
        project_path = root_dir / project_name
        if project_path.is_dir() and not project_name.startswith(('.', '_')):
            print(project_path)
            subprocess.check_call('python -m unittest'.split(), cwd=project_path)
