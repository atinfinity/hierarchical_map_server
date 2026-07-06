# hierarchical_map_server (monorepo)

ROS 2 Jazzy で広域エリア(例: 500m × 500m @0.05m)を Nav2 + AMCL で
ナビゲーションするための、**多重解像度タイル地図**パッケージ群のモノレポ。

広域地図をグリッド状のタイルに分割して管理し、動的に現在地付近だけをロード・
配信することで、地図全体をメモリに載せずに大規模エリアを走行できる。

## デモ

![hierarchical_map_server demo](docs/hier_demo.gif)

TurtleBot3 Gazebo での走行デモ。**低解像度の全域地図**(global costmap 用)の
上をグローバルプランナが計画し、**高解像度窓**(シアンの枠, AMCL 用)がロボットに
追従してスライドする。グローバル経路(オレンジ)は高解像度窓の外まで全域に
伸びており、**窓に縛られず全域で経路計画できる**ことが分かる(案3の要点)。
動画版: [`docs/hier_demo.mp4`](docs/hier_demo.mp4)。

## パッケージ構成

| パッケージ | 役割 |
|---|---|
| [`tile_map_server`](tile_map_server/) | 高解像度タイルを現在地周辺で結合して配信するスライディングウィンドウ地図サーバ(AMCL・costmap 用) |
| [`hierarchical_map_server`](hierarchical_map_server/) | タイルから低解像度の全域地図を生成し、global costmap に供給。高解像度窓の外のゴールへも全域経路を計画可能にする(`tile_map_server` に依存) |

依存関係は `hierarchical_map_server` → `tile_map_server`。`hierarchical_map_server`
は `tile_map_server` のコアライブラリ(タイルセット読込・PGMローダ)を再利用する。
各パッケージの詳細・アーキテクチャ図は各ディレクトリの README を参照。

## 設計の概要

- **tile_map_server(スライディング窓)**: 現在地を中心とする `N×N` タイルを
  1枚の `OccupancyGrid` に結合して `/map`(transient_local)に配信。ロボットが
  タイル境界を越えると窓を再センタリングする。全タイルは単一のグローバル原点で
  座標系を共有するため、窓が切り替わっても AMCL の自己位置推定は連続する。
- **hierarchical_map_server(階層地図)**: 全タイルを起動時にダウンサンプルして
  低解像度の全域地図を1枚生成し `/map_global_lowres` に配信。global costmap は
  これで全域を計画し、高解像度窓は AMCL 専用、精密な障害物回避は
  local costmap(/scan)が担う二重 costmap 構成。

## ビルド

```bash
# 例: colcon ワークスペースの src/ 以下にクローン
mkdir -p ~/nav_ws/src && cd ~/nav_ws/src
git clone https://github.com/atinfinity/hierarchical_map_server.git
cd ~/nav_ws
colcon build
source install/setup.bash
```

依存順(`tile_map_server` → `hierarchical_map_server`)は colcon が自動解決する。

## テスト

```bash
colcon test --packages-select tile_map_server hierarchical_map_server
colcon test-result --verbose
```

- `tile_map_server`: 単体テスト(タイルインデックス・ヒステリシス・結合・キャッシュ)
- `hierarchical_map_server`: 単体テスト(ダウンサンプリング・全域組み立て)
- 両パッケージとも TurtleBot3 Gazebo での結合走行テスト launch を同梱。

## 対象環境

- ROS 2 Jazzy
- Nav2(amcl / costmap_2d / planner)
- 検証: TurtleBot3(Gazebo / gz sim)

## ライセンス

Apache-2.0
