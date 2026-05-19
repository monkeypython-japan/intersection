# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Workflow                                                                                                    
- 会話は全て日本語で行う。                                                            
- すべての変更はブランチを作成してから行う。
- 曖昧な点はユーザに質問する。
-  `main` への直接コミットは禁止。
- ブランチ上で変更を実施したら、ユーザーがテストを行う。
- テスト完了後、Claude がコミットして `main` へマージする。
     
## Project Overview

歩行者が横断歩道ですれ違う状況をシミュレーションする Python 製デスクトップアプリ。多数の歩行者の動きをリアルタイムに表示する。

- 仕様書：`intersection-sepc.md`（Claude は読み取り専用。ユーザーが不定期に更新する）
- 言語：Python（オブジェクト指向）
- 外部ライブラリを追加する場合は、目的と主な機能を説明してからユーザーの承認を得る
- `requirements.txt` を常に最新に保つ

## Setup

tkinter を有効化するため、事前に以下をインストール（Homebrew Python 使用時）：

```bash
brew install python-tk@3.14
```

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Development Commands

```bash
# アプリ起動
python main.py

# テスト実行
python -m pytest

# 単一テスト実行
python -m pytest tests/test_pedestrian.py
```

## Architecture

### 主要クラス

| クラス | 役割 |
|--------|------|
| `Boundary` | 始点・方向ベクトル・長さで定義される境界線。開始線とゴール線を兼ねる |
| `Pedestrian` | 位置・速度・ゴール線を保持。前方 45° 扇形を 3 分割して混雑の少ない方向に移動ベクトルを調整し、衝突を回避する |
| `Crowd` | 同じ開始線/ゴール線を共有する歩行者の集合。歩行者数は外部パラメータで制御 |
| `Intersection` | 50m×50m の平面。境界線と群衆を管理し、シミュレーションのライフサイクルを制御する |

### シミュレーションループ

- 0.1 秒間隔で全歩行者の位置・移動ベクトルを更新し再描画
- 全歩行者がゴール線に到達したら終了

### デフォルト設定

- 境界線 2 本：`(10,10) 方向(0,1) 20m` と `(40,10) 方向(0,1) 20m`
- 2 つの群衆がそれぞれの境界線を開始線/ゴール線として逆方向に横断する

### UI

スライダ（歩行者数）、設定ボタン、開始ボタンで構成。
