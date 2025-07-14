#!/bin/bash
set -e

echo "ROS 2パッケージのバージョン一覧を確認します..."

versions=$(grep -h '<version>' src/*/package.xml | sed -E 's/.*<version>(.*)<\/version>.*/\1/' | sort | uniq)

if [ "$(echo "$versions" | wc -l)" -ne 1 ]; then
  echo "Inconsistent package.xml versions found:"
  grep -H '<version>' src/*/package.xml
  exit 1
fi

# セマンティックバージョン形式かチェック
if ! [[ "$versions" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "Version $versions is not in semantic versioning format (MAJOR.MINOR.PATCH)"
  exit 1
fi

echo "All package.xml versions are consistent and in semantic versioning format: $versions"
