#!/usr/bin/env python3
"""
YAMLパラメータ定義から型・説明付きドキュメントを自動生成するスクリプト。
RST/Markdown両対応（拡張しやすい）。
"""
import yaml
import os

SRC = os.path.join(os.path.dirname(__file__), '../config/sim_params.yaml')
DST = os.path.join(os.path.dirname(__file__), '../docs/param_reference.md')

def parse_yaml_with_meta(yaml_path: str) -> list[dict]:
    """
    Parse a YAML file with parameter metadata and return a list of parameter dicts.
    Args:
        yaml_path (str): Path to the YAML file.
    Returns:
        list[dict]: List of parameter dictionaries with keys: key, value, type, desc.
    """
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    result = []
    for k, v in data.items():
        if isinstance(v, dict) and all(isinstance(val, dict) and 'value' in val for val in v.values()):
            # ネスト: reward, physics など
            for subk, subv in v.items():
                result.append({
                    'key': f'{k}.{subk}',
                    'value': subv.get('value'),
                    'type': subv.get('type', ''),
                    'desc': subv.get('desc', '')
                })
        else:
            # トップレベル
            comment = ''
            if isinstance(v, str) and '#' in v:
                v, comment = v.split('#', 1)
            result.append({
                'key': k,
                'value': v if not isinstance(v, dict) else '',
                'type': 'str',
                'desc': comment.strip() if comment else ''
            })
    return result

def write_markdown(param_list: list[dict], out_path: str) -> None:
    """
    Write parameter documentation as a Markdown table.
    Args:
        param_list (list[dict]): List of parameter dictionaries.
        out_path (str): Output file path for the Markdown file.
    """
    with open(out_path, 'w') as f:
        f.write('# Simulation Parameter Reference\n\n')
        f.write('| Name | Type | Default | Description |\n')
        f.write('|------|------|---------|-------------|\n')
        for p in param_list:
            f.write(f"| `{p['key']}` | {p['type']} | {p['value']} | {p['desc']} |\n")
    print(f'Wrote parameter reference: {out_path}')

if __name__ == '__main__':
    """
    Main entry point: parses the YAML and writes the Markdown documentation.
    """
    params = parse_yaml_with_meta(SRC)
    write_markdown(params, DST) 