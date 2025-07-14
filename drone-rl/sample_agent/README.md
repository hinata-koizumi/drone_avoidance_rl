# Sample CartPole Agent

このディレクトリは「バカでも動く」サンプルとして、GymnasiumのCartPole環境で動作確認・デバッグ用のエージェントを提供します。

## 目的
- 学習がうまくいかない場合、「学習側」か「環境側」かの切り分けを容易にする
- RLパイプラインやGym APIの動作確認
- ROS 2やGazeboを使わず、Gymだけで完結するサンプル

## 内容
- `sample_cartpole_agent.py` :
    - ランダム行動エージェント
    - 直進（常に右）エージェント
    - CartPoleでの学習・評価ループ

## 学習・チェックポイント保存

```bash
python sample_cartpole_agent.py
```
- `linear_policy.pt`（モデル）と `linear_policy_state.pkl`（状態）が自動保存されます。
- 途中で停止しても、再開可能です。

## 学習再開

```bash
python sample_cartpole_agent.py  # resume=True で再開するように編集、または関数を直接呼び出し
```
- コード内の `train_linear_policy_agent(episodes=..., resume=True)` を有効にしてください。

## 再生確認（保存済みモデルで再現）

```bash
python replay_linear_policy.py
```
- 保存済みモデル・状態でエピソードを再生し、可視化・ログ出力します。

## 備考
- 乱数シード・エピソード進行状況も保存されるため、再現性の高いデバッグが可能です。
- モデルや状態ファイルは `linear_policy.pt`, `linear_policy_state.pkl` です。

## 使い方
```bash
cd src/sample_agent
python sample_cartpole_agent.py
```

## 拡張例
- 他のGym環境（例: MountainCar, LunarLander）での動作確認
- 簡易的なQ学習やDQNの実装追加

---
このサンプルでRLパイプラインの基本動作を確認し、問題があれば「学習側」か「環境側」かを切り分けてください。 