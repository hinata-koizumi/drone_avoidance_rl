import os
import requests
import time

# 🔐 環境変数から GitHub トークンを取得（直接コードに書かないように）
TOKEN = os.environ.get("GITHUB_TOKEN")
OWNER = "hinata-koizumi"
REPO = "drone_avoidance_rl"

if not TOKEN:
    raise RuntimeError("❌ GITHUB_TOKEN 環境変数が未設定です。先に `export GITHUB_TOKEN=...` を実行してください。")

HEADERS = {
    "Authorization": f"token {TOKEN}",
    "Accept": "application/vnd.github+json"
}

def get_all_workflow_runs():
    print("🔍 Getting workflow runs...")
    runs = []
    page = 1
    while True:
        url = f"https://api.github.com/repos/{OWNER}/{REPO}/actions/runs?per_page=100&page={page}"
        response = requests.get(url, headers=HEADERS)
        response.raise_for_status()
        data = response.json()
        page_runs = data.get("workflow_runs", [])
        if not page_runs:
            break
        runs.extend(page_runs)
        page += 1
    return runs

def delete_log(run_id):
    url = f"https://api.github.com/repos/{OWNER}/{REPO}/actions/runs/{run_id}/logs"
    response = requests.delete(url, headers=HEADERS)
    if response.status_code == 204:
        print(f"✅ Deleted logs for run {run_id}")
    else:
        print(f"⚠️ Failed to delete logs for run {run_id}: {response.status_code} - {response.text}")

def main():
    runs = get_all_workflow_runs()
    print(f"📦 Found {len(runs)} workflow runs")
    for run in runs:
        run_id = run["id"]
        workflow_name = run["name"]
        print(f"🗑 Deleting logs for run ID {run_id} ({workflow_name})")
        delete_log(run_id)
        time.sleep(0.5)  # レートリミット対策

if __name__ == "__main__":
    main()
