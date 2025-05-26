import os
import yaml
from collections import defaultdict

rosdep_dir = os.path.join(os.path.dirname(__file__), '../rosdep')
all_keys = defaultdict(list)

for fname in os.listdir(rosdep_dir):
    if fname.endswith('.yaml'):
        path = os.path.join(rosdep_dir, fname)
        with open(path, 'r') as f:
            try:
                data = yaml.safe_load(f)
                if not isinstance(data, dict):
                    print(f"[ERROR] {fname} is not a dict at top level.")
                    exit(1)
                for k in data.keys():
                    all_keys[k].append(fname)
            except Exception as e:
                print(f"[ERROR] Failed to parse {fname}: {e}")
                exit(1)

# 重複キー検出
for k, files in all_keys.items():
    if len(files) > 1:
        print(f"[WARN] Key '{k}' is defined in multiple files: {files}")

print("rosdep YAML consistency check complete.") 