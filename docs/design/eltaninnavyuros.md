# eltanin_ros 詳細設計: eltanin をコアとする ROS 2 ナビゲーションスタック

- 対象リポジトリ: `eltanin_ros` (`dev` ブランチ / コミット 0 件 / `main` 未作成)
- 参照リポジトリ: `eltanin` (`dev` / `52bf313`)、`navyu` (`main` / `56978ff`)、`kachaka-api`
- 前工程: `.plait/00-requirements.md` (要件定義 版 5)
- 作成日: 2026-07-31
- フェーズ: 詳細設計 (実装は後続フェーズ)
- 状態: **実装着手可。未確定事項は §11 に残る 4 件のみで、いずれも M4 以降に判断を遅らせられる**

本文書は要件定義の未確定論点 (Q-1 〜 Q-16) をすべて決着させ、実装を分割可能な単位に落とすことを
目的とする。要件定義の記号 (C-n / N-n / E-n / F-n / D-n / Q-n / R-n / M-n / NF-n) をそのまま使う。

---

## 1. この設計が解く問題

`eltanin` は `map_io` から `sim` までの全系が **ROS なしで 1 プロセスの閉ループとして回ることを
実証済み**である (`docs/integration-design.md`)。欠けているのは ROS 2 の入出力と、それに伴う
**時間・座標・失敗経路の管理**だけである。

したがって設計の主題は「ナビゲーションをどう作るか」ではなく、次の 3 つになる。

1. **境界をどこに引くか。** D-10 の判定基準 (`eltanin` = 「この値が与えられたとき何をすべきか」、
   `eltanin_ros` = 「値が得られたか、いつ得られたか」) を、具体的なノード・ファイル・型に落とす。
2. **`eltanin` の型要求 (C-6: `plan()` / `smooth()` / `limit()` がコストマップの実体を要求する) を
   満たすプロセス境界を引く。** これは通信構成の自由度を実質的に決める。
3. **`navyu` の ROS 層欠陥 (N-1 〜 N-13) を構造で再発不能にする。** パラメータ調整や規約ではなく、
   「その形では書けない」状態にすることを優先する。

---

## 2. この設計フェーズで確定した決定事項

要件定義の D-1 〜 D-11 に加える。

| # | 決定 | 対応する論点 |
|---|---|---|
| **D-12** | **段間は topic pipeline、Action は外側の 2 箇所だけ** (`NavigateToPose` と `ComputePathToPose`)。系譜は nav2 ではなく Autoware に寄せる | Q-1 |
| **D-13** | **全ノードを `rclcpp_components` として実装し、既定の bringup は単一 `ComposableNodeContainer` + intra-process comms**。個別プロセス起動も launch 引数で選べる | Q-1 |
| **D-14** | **膨張は `global_costmap` が単独で所有する** (D-6 維持)。`local_map` は 3 値 occupancy を 1 本だけ publish し、膨張版を併せて出さない | Q-12 |
| **D-15** | **`local_path_planner` (DWA) の障害物評価は非膨張 local map への厳密フットプリント判定 (E-1) で行う。** 距離場 (E-8) は「横方向余裕が実測で不足したとき」に限って後から足す | Q-12 / Q-13 / Q-14 |
| **D-16** | **local planner は 2 段で導入する。** 段 1 = 窓切り出し + 速度プロファイル付与 (回避なし)、段 2 = DWA による局所回避 | Q-13 |
| **D-17** | **ゴール最終接近 (減速 + 最終 yaw 合わせ) を `eltanin` 側に追加する** (**E-10** として新設)。`xy_goal_tolerance` / `yaw_goal_tolerance` を導入する | Q-6 |
| **D-18** | **`local_map` は `StaticLayer` を持つ。** 既定は `NO_INFORMATION`、静的地図で埋めたうえで raytrace の自由 / 障害物を重ねる | §5.5 で導出 |
| **D-19** | **E-6 (`Layer` への ROI 追加) を M4 のクリティカルパスから外す。** global の更新契機は再計画時のみとし、差分矩形は `global_costmap` ノードが自分で追跡する。E-6 は破壊的変更なので実装順序の最後に置く | Q-3 / R-13 |
| **D-20** | **内部トピックは生コスト値 (`uint8`) を運ぶ独自メッセージ、可視化は別トピックで `nav_msgs/OccupancyGrid`**。両者の変換規則は 1 箇所 (`eltanin_ros_common`) に置く | Q-4 |
| **D-21** | **内部の速度指令は `geometry_msgs/TwistStamped`、最終出力だけ `geometry_msgs/Twist`**。staleness 判定にタイムスタンプが必要で、kachaka は `Twist` しか受け付けないため | Q-5 |
| **D-22** | **実機の指令出力は既定で無効。** `collision_predictor` の `~/enable_output` サービスで明示的に有効化する。ドック上での意図しない前進 (R-5) を publish 開始そのものから切り離す | F-8 / R-5 |
| **D-23** | **距離場専用ノードを作らない。** 導入する場合は `local_map` が local 窓の距離場を併せて出す (`costmap-design.md` §15.4 の第一候補)。global 側は静的なので 1 回計算すれば済む | Q-14 |
| **D-24** | **`eltanin_vendor` はワークスペース内の隣接 `../eltanin` を既定の参照元とし、`ExternalProject_Add` でビルド・インストールする。** git URL + 固定リビジョンへの切り替えは CMake オプションで可能にする | Q-10 |
| **D-25** | **vendor ビルドは `RelWithDebInfo` (= `NDEBUG` 定義 = `assert` 無効) を既定とする。** 事前条件の検査は `eltanin_ros` 側の入力検証と `create()` の `nullopt` 検査で行う | Q-8 |
| **D-26** | **オーケストレータの状態は明示的な enum + 遷移表で表現する。** BT エンジンは導入しない | Q-15 |
| **D-27** | **スキャンの角度フィルタは既定 `nullopt` (全周)。マーキング最大レンジとクリアリング最大レンジを分け、クリアリングは既定 3.0 m** | Q-16 |
| **D-28** | **`main` は S0 の初期コミット (ビルド基盤 + 本設計文書) の直後に作る。** 以降は `dev` から `main` への PR 経由 | Q-11 |

### 却下した案とその理由 (詳細は §4.4 / §10)

| 却下案 | 理由 |
|---|---|
| 各段を nav2 風の Action サーバにする | `eltanin` に pluginlib 相当の抽象がない (C-6 / C-7) ため「Server」の名前の根拠が伴わない。周期段に Action を置くと preempt レイテンシが安全性の属性になる (F-7) |
| モノリシックな 1 ノード | D-6 のノード構成指示に反し、`navyu` の置き換えとしての責務分離の目的を果たさない |
| `local_map` が非膨張版と膨張版の 2 枚を出す | 同一観測から派生した 2 枚のどちらが正本か曖昧になり、値域変換経路も 2 本になる |
| DWA の障害物項を観測点集合への最近接距離で評価する | 候補数 × 展開点数 × 観測点数 になり、EDT か空間分割が実質必須になる (§6.4) |
| 距離場専用ノード | 全域 `float` 距離場 64 MB を topic に流すか、ローカル窓サイズの出力になって hop が 1 増えるだけ (`costmap-design.md` §15.4) |

---

## 3. 全体アーキテクチャ

### 3.1 ノードとデータフロー

```
                         RViz / CLI
                             │ NavigateToPose (Action)
                             ▼
                    ┌──────────────────┐
                    │    navigator     │  状態機械 (D-26) + eltanin の
                    │  (orchestrator)  │  NavigationSupervisor (E-9) を駆動
                    └──────────────────┘
                       │            │
   ComputePathToPose   │            │  ~/enable (Service)
       (Action)        ▼            ▼
        ┌──────────────────────┐  ┌──────────────────────┐
        │ global_path_planner  │  │  local_path_planner  │
        │  A* + smooth         │  │  窓切り出し + 速度付与 │
        │  (eltanin::planner)  │  │  + DWA (eltanin)     │
        └──────────────────────┘  └──────────────────────┘
              │  /global_path            │  /local_trajectory
              │  (nav_msgs/Path)         │  (eltanin_msgs/Trajectory2D)
              │        └─────────────────┤
              │                          ▼
              │                 ┌──────────────────────┐
              │                 │    path_follower     │  PurePursuit + GoalApproach
              │                 │  (eltanin::control)  │  (E-3 / E-10)
              │                 └──────────────────────┘
              │                          │  /cmd_vel_raw (TwistStamped)
              │                          ▼
              │                 ┌──────────────────────┐
              │                 │ collision_predictor  │  VelocityLimiter (E-1)
              │                 │  + watchdog (F-8)    │  ★ 単一の出力所有者
              │                 └──────────────────────┘
              │                          │  /cmd_vel (Twist)
              │                          ▼
              │                    robot / simulator
              │
    ┌─────────┴────────────┐        ┌──────────────────────┐
    │   global_costmap     │◀───────│      local_map       │
    │  static + obstacle   │ /local │  static + raytrace   │
    │  + inflation         │  _map  │  3 値・膨張なし       │
    └──────────────────────┘        └──────────────────────┘
       │                              │        │
       │ /global_costmap              │        └──▶ collision_predictor
       │ /global_costmap_updates      └──▶ local_path_planner
       ▼                                          (自ノード内で判定モデルを作る)
  global_path_planner

  入力: /map (OccupancyGrid, transient_local) → static
        /scan (LaserScan, SensorDataQoS)      → local_map
        /tf, /tf_static                        → 全ノード
```

**この図の要点は 3 つある。**

1. **`collision_predictor` が `/cmd_vel` の単一の所有者である** (N-1 / F-8)。Action の内側になく、
   `navigator` の状態にも `path_follower` の生死にも依存せずに周期を回す。上流のどれかが期限切れなら
   ゼロ指令を出す。「何も publish しない」「古い指令を再送する」経路をコードに持たない。
2. **`/local_map` は 1 本で、2 人の消費者がそれぞれ自分の判定モデルを作る** (D-14 / D-15)。
   `collision_predictor` は厳密フットプリント判定、`local_path_planner` は DWA の候補評価に使う。
   膨張は `global_costmap` にしかない。
3. **`global_costmap` → `global_path_planner` は再計画のときだけ効く経路である。**
   `local_map` → `global_costmap` の反映も再計画契機のときだけ行う (D-19)。制御周期には乗らない。

### 3.2 なぜ段間を topic にしたか (Q-1 の決着)

| 段 | 性質 | 採る境界 | 根拠 |
|---|---|---|---|
| ゴール受付 | 1 回 / 数十秒。キャンセル・進捗・結果が意味を持つ | **Action** | F-10 / N-5 |
| global 経路生成 | 0.147 s (`-O2`) / 1.48 s (`-O0`) の request/response | **Action** | キャンセルが意味を持つ長さ |
| local 経路生成 → 追従 → 制限 | 連続 10 〜 20 Hz | **topic** | 周期ごとの Action は不適。長寿命 Action にすると preempt が安全経路に混ざる |
| コストマップ / local map | 連続 publish | **topic** | Action 不要 |

`navyu` の欠陥のうち **N-1 (ウォッチドッグ不在) と N-2 (購読状態への書き戻し) は「誰が `/cmd_vel` を
所有するか」が曖昧だったことに帰着する。** 最終段を Action の外の常時稼働ノードに固定することが、
この 2 つに対する構造的な対策である。

### 3.3 実行形態 (D-13)

- 全ノードは `rclcpp_components::RegisterNodeMacro` で登録する。**`navyu` は component 登録しながら
  どの launch も使っていなかった** (§2.4-6)。使う。
- 既定の bringup は単一 `ComposableNodeContainer` (`component_container_mt`) + `use_intra_process_comms`。
  120×120 セルの local map を 10 Hz で流すコストは intra-process ならゼロコピーになる。
- **「単一スレッド実行だから安全」に依存しない** (NF-3 / N-7)。次を全ノードの規約とする。
  - 購読と timer は用途ごとに `MutuallyExclusive` callback group に分ける。
  - ノード間で共有する状態 (最新スキャン / 最新コストマップ / 最新経路 / 最新指令) は
    **`std::mutex` で保護した 1 つの構造体にまとめ、コールバックはコピーを取って抜ける。**
    ホットループの中で lock を保持しない (NF-6)。
- **時計は必ず `node->get_clock()` を使う** (NF-4)。`create_wall_timer` と `rclcpp::Clock()` の直接構築を
  禁止し、`node->create_timer(period, cb, group)` に統一する。`use_sim_time` がそのまま効く。
- **tf lookup はタイムアウト 0 で行い、失敗はその周期の失敗として扱う** (N-3 への対策)。
  高レート timer 内で 0.5 s のブロッキング待ちをする経路を作らない。`tf2_ros::Buffer` には
  `tf2_ros::CreateTimerROS` を設定する。

### 3.4 フレームと時刻の規約

| 項目 | 規約 |
|---|---|
| フレーム | `map_frame` / `odom_frame` / `base_frame` をパラメータで持つ。**コードに `"map"` を直書きしない** (NF-7 / N-13) |
| スキャンのフレーム | **受信メッセージの `header.frame_id` を使う** (C-16)。`laser_frame` を定数として埋めない |
| 自己位置 | `map_frame → base_frame` の tf のみ。**自前で推定しない** (D-8 / C-1) |
| 地図 origin | `info.origin` の **yaw が `1e-6` を超えたら受理せず、frame / resolution / サイズを含む 1 行のエラーを出す** (C-2 / F-1) |
| 時刻の基準 | 各データの `header.stamp`。ノード内で `now()` と比較して staleness を判定する |

---

## 4. パッケージ構成

```
eltanin_ros/
├── docs/design/eltaninnavyuros.md   ← 本文書
├── eltanin_ros/                     メタパッケージ (ament_package のみ)
├── eltanin_vendor/                  eltanin のビルド・インストール (D-24)
├── eltanin_msgs/                    msg / action (interface only)
├── eltanin_ros_common/              変換層・時計・ウォッチドッグ・tf・パラメータ検証
├── eltanin_costmap/                 local_map, global_costmap の 2 ノード
├── eltanin_planner/                 global_path_planner, local_path_planner の 2 ノード
├── eltanin_controller/              path_follower, collision_predictor の 2 ノード
├── eltanin_navigator/               orchestrator
├── eltanin_simulator/               simple_simulator
└── eltanin_bringup/                 launch / config / rviz / map (コードなし)
```

10 パッケージ。`navyu` の 8 に対し `eltanin_vendor` と `eltanin_msgs` が増え、
`navyu_utils` の 4 ライブラリが `eltanin_ros_common` の 1 つに縮む。

### 4.1 分割の基準

- **`eltanin_ros_common` を独立させる理由**: 変換層は M1 の受け入れ条件 (往復整合・退化ケース・
  不正入力の拒否) の対象であり、**ノードを 1 つも起動せずに `colcon test` で回せる単位**である。
  §2.7 の不整合をすべてここに閉じ込め、他のパッケージが同じ変換を書けないようにする。
- **`eltanin_msgs` を独立させる理由**: `rosidl` の生成物に依存するパッケージが 6 つあり、
  循環を避けるには interface 専用パッケージが要る。
- **ノードを 2 つずつ束ねた理由**: `local_map` と `global_costmap` は同じレイヤ機構を使い、
  `path_follower` と `collision_predictor` は同じ機体プロファイルを読む。
  package.xml の依存が一致するものを束ね、一致しないものを分けている。
- **`eltanin_bringup` にコードを置かない**: `navyu_navigation` と同じ形。ただし
  **パラメータの重複定義を作らない** (N-6 / F-14)。§7 に構成を示す。

### 4.2 `eltanin_vendor` (F-15 / D-24)

```cmake
# 概略
set(ELTANIN_VENDOR_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../eltanin"
    CACHE PATH "eltanin source tree")
set(ELTANIN_VENDOR_GIT_URL "" CACHE STRING "use a git checkout instead when non-empty")
set(ELTANIN_VENDOR_GIT_TAG "" CACHE STRING "revision for ELTANIN_VENDOR_GIT_URL")
set(ELTANIN_VENDOR_BUILD_TYPE "RelWithDebInfo" CACHE STRING "")

ExternalProject_Add(eltanin_external
  # SOURCE_DIR または GIT_REPOSITORY/GIT_TAG のどちらか
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE=${ELTANIN_VENDOR_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DELTANIN_BUILD_TESTS=OFF
    -DELTANIN_BUILD_EXAMPLES=OFF
  BUILD_ALWAYS ON)   # 並行開発 (A-5) で隣接ソースの変更を取りこぼさないため
```

- 下流は `find_package(eltanin REQUIRED)` + `target_link_libraries(... eltanin::core ...)`。
  ament が `AMENT_PREFIX_PATH` を `CMAKE_PREFIX_PATH` に流すのでそのまま解決する。
- **`eltanin` 側に ROS 由来のファイルを 1 つも追加しない** (F-15 / C-14)。`package.xml` も置かない。
- `add_subdirectory` 案は却下した。`eltanin` の `install(EXPORT eltaninTargets)` が
  ament のインストール空間に混ざり、`eltanin_vendor` 自身のターゲットと同じ export セットを
  共有できないためである。
- **`BUILD_ALWAYS ON` の代償**: `colcon build` のたびに `eltanin` の CMake 再構成が走る。
  実測はクリーン 11.9 s に対し no-op 0.9 s で、並行開発中はこれを受け入れる。
  リリース時は git URL + 固定リビジョンに切り替える。
- `project(eltanin_vendor LANGUAGES CXX)` とし、**`CMAKE_CXX_COMPILER` を sub-build に転送する。**
  `NONE` では `CMAKE_CXX_COMPILER` が定義されず、colcon が選んだコンパイラと sub-build が
  食い違いうる。`CMAKE_TOOLCHAIN_FILE` と ccache は転送していない (クロスコンパイルも ccache も
  現状の要件になく、空値を渡す分岐を増やさない)。

**`CMAKE_POSITION_INDEPENDENT_CODE=ON` が必須である理由 (実装フェーズで実測)**

`eltanin` は全モジュールを `STATIC` で宣言し、`CMAKE_POSITION_INDEPENDENT_CODE` を自分では
設定しない。一方**全ノードは `rclcpp_components` = 共有ライブラリ**である (§3.3 / D-13)。
非 PIC のアーカイブを共有オブジェクトにリンクすると `R_X86_64_PC32` の再配置エラーになる。
実測では `CMAKE_BUILD_TYPE` 空 (`-O0`) で `core` / `map` / `map_io` / `planner` の 4 つ、
`RelWithDebInfo` で `map` / `map_io` の 2 つが失敗した。

**失敗する数は build type とコンパイラに依存し、残りは「たまたま」通る。**これが問題の本体である。
`eltanin::sensor` だけを使う最初のノードでは表面化せず、後のノードで突然リンクが落ちる。
最適化設定を変えただけで症状が出たり消えたりもする。したがって PIC は
**指定漏れを全モジュールについて検出する仕組みごと**必要である。

**リンク検証 (`eltanin_vendor/test/link_check/`)**

`find_package(eltanin)` して**モジュールごとに 1 つの `SHARED` ライブラリ**を作り、
そのモジュールだけを `WHOLE_ARCHIVE` で全オブジェクト引き込む。`DEPENDS eltanin_external` つきの
入れ子 `ExternalProject_Add` として構築する (`eltanin` の install は本パッケージの build ステップで
起きるので、configure 時点では `find_package` が解決しない)。

- **`WHOLE_ARCHIVE` が要る理由**: `SHARED` に静的アーカイブを普通にリンクすると参照された
  メンバだけが引き込まれるので、非 PIC のオブジェクトが混ざっていても表面化しない。
- **1 つの `SHARED` に 8 モジュールを並べる形は成立しない** (実測)。`eltanin::map` が
  `eltanin::core` を PUBLIC 依存するため同じ item が直接と推移で 2 回現れ、CMake が
  「link item specified without any feature ... has already occurred with the feature
  'WHOLE_ARCHIVE'」で generate に失敗する。`LINK_LIBRARY_OVERRIDE` で全 item に強制すると
  `libeltanin_core.a` が 2 回 whole-archive され重複定義でリンクが落ちる。
  モジュールごとに 1 つなら whole-archive されるアーカイブは常に 1 つで、
  失敗メッセージにどのモジュールが非 PIC かが出る。
- **`-Wl,--no-undefined` を付ける。** 全オブジェクトを引き込んだ状態で未解決シンボルが残らない
  ことを要求する = `eltaninConfig.cmake` の `find_dependency(Eigen3)` / `find_dependency(yaml-cpp)` と
  export された依存が消費側 prefix で実際に足りていることの検査になる。
- **ctest にしない。** リンク可能性は「テストの失敗」ではなく「ビルドの失敗」であるべきで、
  `colcon build` が赤くなる形にしたい。`colcon test` は失敗しても exit 0 なので担保としても弱い。
- この入れ子プロジェクトだけ `cmake_minimum_required(VERSION 3.24)` である
  (`WHOLE_ARCHIVE` と `LINK_LIBRARY_OVERRIDE_<item>` が 3.24 以降)。Jazzy / Noble は 3.28。

**`colcon` が `src/eltanin` を勝手にビルドする — `--packages-ignore eltanin` が必須**

colcon は `package.xml` を持たない素の CMake プロジェクトも **`cmake` 型パッケージとして認識する**
(`colcon list` が `eltanin  src/eltanin  (cmake)` を出す)。対策なしでは `eltanin` が 2 回ビルドされ、

1. **非 PIC の第 2 の prefix `install/eltanin`** ができる。`find_package(eltanin)` から見えうるうえ、
   どちらが勝つかは `CMAKE_PREFIX_PATH` の順序に依存する = いつか静かに非 PIC 側を引く
2. `ELTANIN_IS_TOP_LEVEL=ON` なので `ELTANIN_BUILD_TESTS` が既定 ON になり、GTest を要求する
3. `eltanin` の 432 テストが `colcon test` に混ざり、`eltanin` 側の CI と二重になる

`colcon build` / `colcon test` の**両方**に `--packages-ignore eltanin` を付け、CI と README の
1 箇所ずつで固定する。`src/eltanin/COLCON_IGNORE` を置く案は C-14 (`eltanin` に ROS 由来の
ファイルを追加しない) に反するため採らない。`vcs import` 後の CI には存在しないので結局
フラグが要る点も同じである。**この対策は開発者の記憶に頼るという弱さが残る。**

**却下: `ament_vendor()` を使う案**

Jazzy には `ament_cmake_vendor_package` があり、`VCS_TYPE path` で隣接ソースも扱え、
コンパイラ / toolchain の転送も済んでいる。それでも採らない。

| 理由 | 内容 |
|---|---|
| install 先が `${CMAKE_INSTALL_PREFIX}/opt/eltanin_vendor` になる | 下流が `find_package(eltanin_vendor)` を先に呼ぶか環境フックを撒く必要が生じ、「下流は `find_package(eltanin)` だけ」が崩れる |
| `BUILD_SHARED_LIBS ON` を既定で強制する | `eltanin` が全ライブラリを明示 `STATIC` で宣言している現状は無害だが、上流の宣言が変わった瞬間に静かに共有ライブラリ化する |
| `path` モードが `file(GLOB_RECURSE "${VCS_URL}/*")` を configure 依存に登録する | 隣接ソースは開発機では作業ツリーであり `.git/` と `build*/` を含む。数万ファイルが configure 依存になり、`BUILD_ALWAYS ON` (0.9 s) より確実に重い |

自前の `ExternalProject_Add` は 50 行程度で、失敗時の診断メッセージを自分で書ける利点もある。

### 4.3 `assert` の扱い (Q-8 / D-25)

`eltanin` は `CMAKE_BUILD_TYPE` を空にして `assert()` を有効に保つ設計 (C-13) だが、
**実機で事前条件違反が `abort()` になるのは、ノードが死んで `/cmd_vel` が止まることを意味する。**
`collision_predictor` が死ねば watchdog も消え、最後の指令のまま走り続ける危険がある
(kachaka のロボット側 watchdog 0.3 s が最後の防波堤になるが、それに頼る設計にはしない)。

したがって `RelWithDebInfo` (assert 無効) を既定とし、**代わりに次を義務づける**。

| 事前条件 | `eltanin_ros` 側の担保 |
|---|---|
| `map.geometry().resolution() > 0` / `cell_count() > 0` | コストマップ構築時に検証し、不正なら起動時エラーで拒否する (F-14) |
| `in_bounds(mx, my)` (`GridMap::operator()` / `MapGeometry::index`) | **ノード側は必ず境界検査付き API (`get` / `set` / `world_to_map` / `world_rect_to_cells`) を使う。** 生の `operator()` を `eltanin_ros` のコードに書かない |
| `create()` の `nullopt` (フットプリント退化 / 非凸 / 原点非包含 / スムーザ重み) | 起動時に検証し、**どのパラメータがなぜ不正かを含む 1 行のエラー**で拒否する |
| `dt > 0` | 実経過時間を計算した直後に検査し、0 以下ならその周期をスキップする |

開発時は `--cmake-args -DELTANIN_VENDOR_BUILD_TYPE=Debug` で assert を有効化できる。
CI は Debug と RelWithDebInfo の両方を回す。

**ただし `ELTANIN_VENDOR_BUILD_TYPE` は assert の有効・無効を完全には支配しない。**
`eltanin` の事前条件検査の相当部分はヘッダ内のテンプレート / `inline` 関数にあり
(`GridMap::operator()` / `MapGeometry::index` / `collision_checker.hpp` の判定)、
**ヘッダ内の `assert` は消費側 TU の `NDEBUG`** = `CMAKE_BUILD_TYPE` に従う。
`ELTANIN_VENDOR_BUILD_TYPE` が支配するのは `eltanin` の `.cpp` に閉じた部分だけである。
CI は両者を同じ値で回すので実害はないが、`-DCMAKE_BUILD_TYPE=Debug
-DELTANIN_VENDOR_BUILD_TYPE=RelWithDebInfo` のような混在では「ヘッダ側の assert だけ生きる」
状態になる。**したがって上の表の担保 (とくに境界検査つき API のみを使う規約) は
build type に関係なく必要である。**

### 4.4 却下: モノリシック構成 / Action サーバ構成

- **モノリシック 1 ノード** は `examples/navigation_loop.hpp` をそのまま載せられる点で最短だが、
  D-6 のノード構成指示に反し、`navyu` の置き換えとして責務分離の目的を果たさない。
  また **`collision_predictor` の独立性 (F-7 / F-19) が構造として担保されなくなる。**
- **各段を Action サーバにする構成** (nav2 風) は nav2 との対応が取りやすいが、
  (a) `eltanin` にプラグイン抽象がないので「Server」の名前の根拠が伴わない (C-6 / C-7)、
  (b) 周期段を長寿命 Action の内側に入れると preempt / cancel のレイテンシが安全性の属性になる、
  (c) `eltanin` の `limit()` がコストマップの実体を要求するため (C-6) Action 境界が
  コストマップの受け渡しを解決しない — の 3 点で採らなかった。

---

## 5. データモデル

### 5.1 新規に定義するメッセージ (D-9 / Q-5)

`eltanin_msgs` に置く。**既存で足りるものは定義しない。**

| ファイル | 内容 | 定義する理由 |
|---|---|---|
| `msg/Costmap.msg` | `std_msgs/Header header` / `MapMetaData2D info` / `uint8[] data` | `eltanin` は `uint8` 0..255。`nav_msgs/OccupancyGrid` の `int8` -1..100 では `LETHAL_OBSTACLE(254)` と `INSCRIBED_INFLATED_OBSTACLE(253)` と `NO_INFORMATION(255)` を区別して運べない |
| `msg/MapMetaData2D.msg` | `float64 resolution` / `uint32 width` / `uint32 height` / `geometry_msgs/Point2D 相当 (float64 origin_x, origin_y)` | **origin に yaw を持たせない。** `MapGeometry` が持たない自由度をメッセージに作ると C-2 の検証を毎回書くことになる |
| `msg/CostmapUpdate.msg` | `std_msgs/Header header` / `uint32 x` / `uint32 y` / `uint32 width` / `uint32 height` / `uint8[] data` | 生値の差分。`map_msgs/OccupancyGridUpdate` は `int8[]` なので生値を運べない。**可視化用には `OccupancyGridUpdate` を別トピックで出し、再定義はしない** |
| `msg/TrajectoryPoint2D.msg` | `geometry_msgs/Pose2D 相当 (float64 x, y, theta)` / `float64 linear_velocity` / `float64 angular_velocity` / `builtin_interfaces/Duration time_from_start` | E-2。`nav_msgs/Path` にも `eltanin::Path` にも速度がない |
| `msg/Trajectory2D.msg` | `std_msgs/Header header` / `TrajectoryPoint2D[] points` | 同上。`header.frame_id` は `eltanin_ros` の関心 (D-10) |
| `msg/NavigationState.msg` | `std_msgs/Header header` / `uint8 state` (**定数 7 値**) / `uint8 outcome` (既定 255) / `string message` / `float64 distance_remaining` / `uint16 replans` | F-10 / F-19 / NF-5。navyu は結果が外から一切観測できなかった (N-5) |
| `action/NavigateToPose.action` | goal: `geometry_msgs/PoseStamped pose` / result: `uint8 outcome` + `string message` + `float64 final_position_error` / feedback: `NavigationState` | ゴール契約。`nav2_msgs` に依存しない (依存を 1 つ減らし、`outcome` を `eltanin` の `NavigateOutcome` に 1:1 で対応させられる) |
| `action/ComputePathToPose.action` | goal: `PoseStamped goal` + `PoseStamped start` + `bool use_start` / result: `nav_msgs/Path path` + `uint8 outcome` + `string message` / feedback: **空** | F-5。計画失敗を「無言」にしない (N-5) |

**合計 8 インターフェース (msg 6 / action 2)。`srv/` は作らない。**

**却下: `srv/SetEnabled.srv`。**当初は `request: bool enabled` / `response: bool success` +
`string message` として設計していたが、**これは `std_srvs/SetBool` と形が完全に一致する**
(`request: bool data` / `response: bool success` + `string message`)。本節冒頭の
「既存で足りるものは定義しない」に反するので削除し、`~/enable_output` と `~/enable` は
`std_srvs/SetBool` を使う。`~/update` と `~/reset` で既に `std_srvs/Trigger` を使うので
依存も増えない。

`ComputePathToPose` の feedback を空のままにするのも同じ立場である。計画は 1 回の呼び出しで
終わり報告すべき中間進捗がない。入れるものができた時点で足す。

**`outcome` の値は `eltanin` の `NavigateOutcome` (11 値) と 1:1 に対応させる。**
`Reached` / `ModelFailed` / `StartGoalFailed` / `PlanFailed` / `PathTooShort` / `NoPath` /
`GoalToleranceFailed` / `ReplanFailed` / `ReplanLimit` / `Stalled` / `StepLimit` の宣言順を
そのまま `OUTCOME_REACHED=0` .. `OUTCOME_STEP_LIMIT=10` とし、ROS 固有の
`OUTCOME_CANCELED=11` / `OUTCOME_TIMEOUT=12` / `OUTCOME_INPUT_STALE=13` を追加する。
**対応表は `eltanin_ros_common` の 1 関数に閉じる。**

- 進行中 (feedback) の値として **`OUTCOME_UNKNOWN=255`** を置き、
  **`outcome` フィールドの既定値にもする** (`uint8 outcome 255`)。
  0..13 を使い切るので sentinel は enum の値域外に置く。`OUTCOME_REACHED=0` を「未確定」に
  流用すると、既定構築したメッセージと進行中の feedback が**成功に見える**。
- `ComputePathToPose` も同じ値集合を使う。計画段が出さない値 (`Stalled` / `ReplanLimit` 等) は
  単に出現しない。集合を分けると変換関数が 2 つになる。
- **定数は `NavigationState.msg` 1 箇所にのみ置く。** rosidl は定数のパッケージ間共有を持たず、
  定数専用 msg は「利用者が現に存在しない型」を作ることになる。他のインターフェースの
  `uint8 outcome` にはコメントで参照先を書き、C++ 側は
  `eltanin_msgs::msg::NavigationState::OUTCOME_*` を参照する。
- **`OUTCOME_*` はこの時点で確定した wire format である。**`NavigateOutcome` は現在
  `examples/navigation_loop.hpp` にしかなく vendor 経由では見えないので、
  1:1 対応をコードで検証できるのは `outcome_conversion.hpp` (S2 後半) と
  enum が `include/eltanin/navigation/supervisor.hpp` に移る S11 (E-9) である。
  **そこで enum の並びを変えるなら、`OUTCOME_*` ではなく対応表を明示的に更新する。**
- `eltanin_msgs/action/NavigateToPose` は `nav2_msgs/action/NavigateToPose` と**同名の別型**である。
  RViz の Nav2 パネルからは駆動できないので、ゴール入力経路 (`/goal_pose` 購読を足すか) は
  §6.7 の `navigator` を実装する時点で決める。

### 5.2 既存メッセージを使うもの

| 用途 | メッセージ |
|---|---|
| 静的地図の入力 | `nav_msgs/OccupancyGrid` (`transient_local`) |
| スキャン | `sensor_msgs/LaserScan` (`SensorDataQoS`) |
| global 経路 | `nav_msgs/Path` |
| 内部の速度指令 | `geometry_msgs/TwistStamped` (D-21) |
| 指令出力 / local planner の有効化 | `std_srvs/SetBool` (§5.1 で `SetEnabled` を却下) |
| 手動トリガ (`~/update` / `~/reset`) | `std_srvs/Trigger` |
| 最終出力 | `geometry_msgs/Twist` (kachaka の制約) |
| コストマップの可視化 | `nav_msgs/OccupancyGrid` / `map_msgs/OccupancyGridUpdate` |
| 予測姿勢の可視化 | `nav_msgs/Path` |
| フットプリントの可視化 | `geometry_msgs/PolygonStamped` |
| オドメトリ (simulator) | `nav_msgs/Odometry` + tf |

### 5.3 コスト値域の変換規則 (Q-4 / D-20)

**変換は `eltanin_ros_common/cost_conversion.hpp` の 4 関数だけに存在する。**

**入力: `OccupancyGrid` (静的地図) → `eltanin::map::Costmap`**

`nav2_map_server` の慣行に倣い、閾値で 3 値に落とす。地図は本来 3 値であり、中間値に意味を与えない。

| 入力 | 出力 |
|---|---|
| `-1` (unknown) | `NO_INFORMATION` (255) |
| `>= occupied_threshold` (既定 65) | `LETHAL_OBSTACLE` (254) |
| `<= free_threshold` (既定 25) | `FREE_SPACE` (0) |
| その間 | `NO_INFORMATION` (255) — **保守側に倒す** |

**出力: `eltanin::map::Costmap` → `OccupancyGrid` (可視化のみ)**

`nav2_costmap_2d` の変換表に倣う。

| 入力 | 出力 |
|---|---|
| `NO_INFORMATION` (255) | `-1` |
| `LETHAL_OBSTACLE` (254) | `100` |
| `INSCRIBED_INFLATED_OBSTACLE` (253) | `99` |
| `FREE_SPACE` (0) | `0` |
| 1..252 | `1 + (cost - 1) * 97 / 251` に線形写像 (1..98) |

**内部トピック: `eltanin::map::Costmap` ⇄ `eltanin_msgs/Costmap` は生値のバイト列コピー。**
**この経路だけが往復で情報を失わない**ので、`colcon test` で往復整合を固定するのはこちら
(M1 の受け入れ条件)。可視化経路は「3 値分類 (`Free` / `Circumscribed` / `Inscribed`) が保たれること」
だけを固定する (`costmap-design.md` §8.1.1 が数値一致を目指さないとしているため)。

### 5.4 姿勢・速度・時刻の変換

| 項目 | 規則 |
|---|---|
| 四元数 → `yaw` | `tf2::getYaw()` を使い、結果を `eltanin::normalize_angle()` に通す |
| `yaw` → 四元数 | `tf2::Quaternion(0, 0, yaw)` |
| `Twist2D` → `Twist` | `linear.x()` と `angular` のみ。**`linear.y()` は捨てる** (差動二輪) |
| `Twist` → `Twist2D` | `linear = (msg.linear.x, 0)` |
| `dt` | **前回のコールバック時刻との実差分。** パラメータの公称周期を使わない (C-8 / navyu の欠陥) |
| `Transform2D` | `TransformStamped` の translation の x,y と `getYaw()` から構築。**z / roll / pitch が閾値を超えたら警告を 1 回だけ出す** (2D 前提が破れていることを黙らせない) |

### 5.5 local map の合成規則 (E-4 / E-5 / D-18)

**この節は「なぜ `local_map` が `StaticLayer` を持つのか」を示す。設計上最も間違えやすい箇所である。**

raytrace で自由セルを計算するなら既定コストは `NO_INFORMATION` でなければならない (E-5)。
`FREE_SPACE` にすると global 側の合成が「センサ視野外の静的障害物」を消す。

ところが **`local_map` が静的地図を含まないと、`collision_predictor` が原理的に動けない。**

- `CostTraversabilityModel::is_obstacle()` は `unknown_is_free == false` のとき
  `NO_INFORMATION` を障害物とみなす。
- kachaka の LiDAR は `base_link` から `(0.156, 0)` 前方、`range_min` 0.1 m。
  **ロボット直下と後方近傍は構造的に未観測になる。**
- フットプリントの inscribed radius は 0.120 m。**未観測セルがフットプリント内に必ず入るため、
  厳密判定は常に `Collision` を返し、ロボットは 1 mm も動けない。**
- `unknown_is_free = true` に逃げると、壁の陰や死角の静的障害物がすべて自由空間になる。**危険。**

したがって `local_map` は `LayeredCostmap` を次の順で組む
(`integration-design.md` §4 のデモも同じ形を採っている)。

```
default_cost = NO_INFORMATION
  1. StaticLayer      静的地図をローカル窓へ再標本化してコピー (格子スナップにより 1:1)
  2. RaytraceLayer    clearing で FREE_SPACE、marking で LETHAL_OBSTACLE (新規レイヤ / E-4+E-5)
  (膨張なし — D-14)
```

**`RaytraceLayer` の合成優先規則:**

| 状況 | 規則 | 理由 |
|---|---|---|
| 同一セルに marking と clearing が立つ | **marking を優先** | 保守側 (E-5) |
| clearing が静的地図の `LETHAL_OBSTACLE` を消そうとする | **既定では消さない** (`clear_static_obstacles`、既定 `false`) | 自己位置誤差で静的壁を消すと `collision_predictor` が壁を見なくなる。動的障害物は静的地図に無いので消せる |
| clearing が `NO_INFORMATION` を消す | 消して `FREE_SPACE` にする | 観測は未知に対する情報の増加 |
| marking が `NO_INFORMATION` を上書き | 上書きする | `ObstacleLayer` の既存契約と同じ (§14.5) |

**窓の管理 (C-11 / C-12):**

- **原点更新と `update()` は必ず同一周期で対にする。** `set_origin()` はセルを動かさない。
- **原点は静的地図の格子にスナップする。** `center_on()` を使わない。
  `navigation_loop.hpp` の `snapped_window_origin` を `eltanin` へ移す (E-9 群 B / §6.1)。
- 窓サイズは既定 6.0 m 角 (120×120 セル)。**根拠はクリアリング有効範囲の上限 3.05 m** であり、
  デモが 6.0 m を採った根拠 (膨張の目減りを差し引いた信頼半幅) とは別である (F-4.1)。

---

## 6. ノード詳細設計

各ノードについて、責務 / 入出力 / 周期 / 内部状態 / 失敗時の挙動 / 主要パラメータを示す。
**「失敗時の挙動」は N-1 〜 N-13 への対策そのものである。**

### 6.1 `local_map` (`eltanin_costmap`)

| 項目 | 内容 |
|---|---|
| 責務 | LiDAR 観測をロボット中心のローカル窓に反映した **3 値 occupancy を 1 本だけ publish する** |
| 購読 | `/map` (`transient_local`)、`/scan` (`SensorDataQoS`)、tf (`map_frame → base_frame`, `map_frame → scan.header.frame_id`) |
| 公開 | `~/local_map` (`eltanin_msgs/Costmap`)、`~/local_map_visual` (`OccupancyGrid`、`publish_visualization` が true のときのみ) |
| 周期 | `update_frequency` 既定 10.0 Hz (スキャン 10.42 Hz の上限に合わせる) |
| 内部状態 | `LayeredCostmap` (static + raytrace)、最新スキャンのコピー、最新 tf |
| 使う eltanin API | `map::LayeredCostmap` / `StaticLayer` / 新規 `RaytraceLayer` / `sensor::project_scan` / 新規 clearing 投影 (E-4) / `MapGeometry::snapped_origin_for` (E-9 群 B) |

**失敗時の挙動:**

| 失敗 | 挙動 |
|---|---|
| `/map` 未受信 | publish しない。`navigator` は `local_map` の staleness で失敗を検出する |
| `/map` の origin yaw ≠ 0 | **起動を続けず地図を拒否**し、frame / resolution / サイズを含むエラーを 1 行出す (C-2) |
| スキャンが `scan_timeout` (既定 0.5 s) より古い | **その周期は raytrace を適用せず、static のみの窓を publish する** (N-4)。古いスキャンを永久に障害物として投影しない |
| tf lookup 失敗 | その周期をスキップし、`throttle` 付きで警告。**窓の原点を動かさない** |
| ロボットが静的地図の外 | 窓を静的地図にクランプし、`window_clamped` を診断に出す |

**受信メッセージを書き換えない** (N-4)。`ScanData` への変換は必ずコピーで行う。

**主要パラメータ:** `update_frequency` / `window_size` (6.0 m) / `resolution` (地図から継承、
不一致なら拒否) / `scan_timeout` / `marking_max_range` (8.0 m) / `clearing_max_range` (3.0 m、D-27) /
`min_range` / `angle_range` (既定なし = 全周) / `clear_static_obstacles` (false) /
`occupied_threshold` / `free_threshold` / `publish_visualization`

### 6.2 `global_costmap` (`eltanin_costmap`)

| 項目 | 内容 |
|---|---|
| 責務 | 全域の膨張コストマップを所有し、**再計画に使える信念**を提供する。local 観測の反映と差分出力を行う |
| 購読 | `/map` (`transient_local`)、`local_map/local_map` |
| 公開 | `~/global_costmap` (`eltanin_msgs/Costmap`、`transient_local`、**更新のたびに全域を流す**)、`~/global_costmap_updates` (`eltanin_msgs/CostmapUpdate`)、`~/global_costmap_visual` (`OccupancyGrid`、`transient_local`) |
| サービス | `~/update` (`std_srvs/Trigger`) — **`navigator` が再計画の直前に呼ぶ** |
| 周期 | **timer で回さない。** 契機は (a) `/map` 受信時の初回構築、(b) `~/update` 呼び出し |
| 内部状態 | `LayeredCostmap` (static + obstacle + inflation)、蓄積した観測点、前回の変化矩形 |

**更新契機を `~/update` に限る理由 (C-3 / D-19):** 4000×4000 = 1600 万セルの `update()` は
0.285 s (`-O2`) かかり、制御周期に乗らない。再計画は数十秒に 1 回なので、そのときだけ回せば足りる。
`local_map` は購読して観測点を蓄積するが、`update()` は呼ばない。

**観測点の蓄積規則 (§2.3 / F-9):**

- `local_map` の `LETHAL_OBSTACLE` セルのうち、**静的地図で `LETHAL_OBSTACLE` でないもの**の中心座標を
  蓄積する。これが「未知障害物を発見した」の定義になる (`integration-design.md` §8)。
- セル線形インデックスを `unordered_set` で重複排除し、初出のときだけ発見順の `vector` に積む。
  出力順は `vector` が決めるので決定的。
- **`local_map` 経由なので投影点は既にセル中心に乗っており、`integration-design.md` §7 の
  「浮動小数点のセル境界ずれで自由空間に偽 `LETHAL` が残る」問題は構造的に発生しない。**
  デモがフィルタを必要としたのは `project_scan` の生の点を直接使っていたためである。

**差分矩形の追跡 (E-6 なしで成立させる / D-19):**

`r = ceil(inflation_radius / resolution)` として、`costmap-design.md` §15.3 の規則を守る。

| ROI | 範囲 | 用途 |
|---|---|---|
| 変化領域 | 今回 `update()` で新規に反映した観測点の外接矩形 ∪ **前回のローカル窓** | 前回の動的障害物を消すため |
| 読み出し | 変化領域 ⊕ `2r` | ⊕`r` の各セルのコストを正しく再計算するため |
| 書き込み (publish) | 変化領域 ⊕ `r` | 窓端の膨張が切れないため |

**現段階では `update()` が全域を再生成するので「読み出し ROI」は自然に満たされる。**
ノードが計算するのは publish する矩形だけである。E-6 が入った後は読み出し / 書き込みの両方が
`Layer` 側の関心になる。**この分離により E-6 を後回しにできる。**

**蓄積するか否かの明示 (F-4.2 / `costmap-design.md` §15.2):**
**蓄積する。** よって「origin 更新でセルデータをシフトしない」の成立前提が破れるが、
**`global_costmap` は origin を一度も動かさない** (静的地図と同じジオメトリで固定) ため
実際には破れない。この理由を実装のコメントに残す。

**失敗時の挙動:** `/map` 未受信なら `~/update` に失敗を返す。`clear_observations` サービスで
蓄積をリセットできる (ゴールごとにリセットするかは `navigator` の判断)。

**主要パラメータ:** `inflation_radius` (0.55) / `cost_scaling_factor` (10.0) / `inflate_unknown` (false) /
`unknown_is_free` (false) / `occupied_threshold` / `free_threshold` / `publish_visualization`

### 6.3 `global_path_planner` (`eltanin_planner`)

| 項目 | 内容 |
|---|---|
| 責務 | start / goal を受け A\* + 平滑化で経路を出す |
| Action サーバ | `~/compute_path_to_pose` (`eltanin_msgs/ComputePathToPose`) |
| 購読 | `global_costmap/global_costmap` (`transient_local`)、`global_costmap/global_costmap_updates` |
| 公開 | `~/global_path` (`nav_msgs/Path`)、`~/global_path_raw` (平滑化前、`publish_raw_path` が true のとき) |
| 使う eltanin API | `planner::plan` / `planner::smooth` / `map::CostTraversabilityModel` |

**失敗の通知 (N-5 / F-5):** `plan()` が `nullopt` を返す原因は 4 つあり、**Action の result で区別する**。

| 原因 | `outcome` |
|---|---|
| start / goal が地図外 | `StartGoalFailed` |
| goal セルが `Free` でない | `NoPath` (「到達不能」) |
| `find_nearest_traversable` が start を救済できない | `StartGoalFailed` |
| 探索が経路を見つけられない | `PlanFailed` |

`eltanin` の `plan()` は現在この 4 つを `nullopt` に潰している。**区別のために `eltanin` 側へ
理由付き戻り値を足すことは今回は行わない** — 呼び出し側が `world_to_map` と `model.classify()` で
同じ検査を先に行えば区別でき、`eltanin` の API を変えずに済む。この重複は
「失敗理由を外に出す」(F-10) ために意図的に受け入れる。

**ゴールの扱い (§2.4-2 への対策):**

- **`header.frame_id` を検査する。** `map_frame` と異なれば tf で変換し、変換できなければ拒否する。
  navyu は無検査で `map_frame` と仮定していた。
- **`orientation` を捨てない。** `plan()` は `goal.yaw` を経路末尾に載せる。
- goal が非 `Free` の場合、**救済しない。** `eltanin` の `plan()` が「ゴールは呼び出し側が求めたもの
  なので動かさず報告する」設計 (`astar_planner.hpp` のコメント) をそのまま外に出す。

**主要パラメータ:** `start_search_radius_cells` (8) / `weight_data` (0.5) / `weight_smooth` (0.3) /
`smoother_tolerance` / `smoother_max_iterations` / `publish_raw_path`

### 6.4 `local_path_planner` (`eltanin_planner`)

**本タスク最大の新規実装 (E-7)。D-16 により 2 段で導入する。**

| 項目 | 内容 |
|---|---|
| 責務 | global 経路から窓を切り出し、**目標速度を付与した Local trajectory** を出す。段 2 で局所回避を加える |
| 購読 | `global_path_planner/global_path`、`local_map/local_map`、tf |
| 公開 | `~/local_trajectory` (`eltanin_msgs/Trajectory2D`)、`~/candidates` (`visualization_msgs/MarkerArray`、段 2 のデバッグ用) |
| サービス | `~/enable` (`std_srvs/SetBool`) |
| 周期 | `update_frequency` 既定 20.0 Hz |

**段 1: 窓切り出しと速度プロファイル (回避なし)**

```
切り出し: ロボット最近傍点から前方 max(lookahead_dist, v * horizon_time) の区間
          + 到達判定のため終端まで残り距離が trim_tail_dist 未満なら終端まで含める
速度:     v_i = min( max_linear_vel,
                     sqrt(max_lateral_accel / |curvature_i|),   // 曲率
                     sqrt(2 * max_decel * remaining_arc_i),      // ゴール減速 (E-10 と同じ式)
                     v_{i-1} + max_accel * dt_i )                // 加速度制限 (前進パス)
          後退パス (i = n-1 .. 0) で減速制限も適用する
```

曲率は 3 点円で求める。**`eltanin` 側に `planner::assign_velocity_profile()` として置く**
(D-10: 「この値が与えられたとき何をすべきか」= `eltanin`)。

**段 2: DWA (D-15 / D-16)**

```
候補: (v, w) を [v_min, v_max] × [-w_max, w_max] の格子で dynamic window に制限してサンプリング
      dynamic window = 現在速度 ± (加速度限界 × sim_period)
展開: integrate_differential_drive() で sim_time 秒ぶん
評価: 衝突除外   = 展開姿勢のいずれかで check_footprint_exact() が Collision → 候補を捨てる (E-1)
      余裕      = フットプリントを clearance_steps 段階に膨らませた多角形で判定し、
                  「何段階まで通るか」を離散的な余裕として使う           ← EDT を使わない代替
      コスト    = w_path * 経路への横方向誤差
                + w_heading * 切り出し終端への向き誤差
                + w_velocity * (v_max - v)
                + w_clearance * (1 - 余裕段数 / clearance_steps)
```

**`w_clearance` を「段階的にフットプリントを膨らませた判定」で作る理由 (D-15 / §2 の却下案):**
連続値の余裕には距離場が必要だが、`clearance_steps` = 3 程度の離散段でも
「障害物すれすれを通る候補」と「余裕のある候補」を区別できる。判定量は
`候補数 × 展開点数 × clearance_steps` 回の `contains_occupied_cell()` で、
1 回のコストはフットプリント AABB (8×5 セル程度) の走査にすぎない。
**距離場 (E-8) は「これで横方向余裕が実測で不足した場合」に導入する** (§11 の未確定事項 U-1)。

**失敗時の挙動:**

| 失敗 | 挙動 |
|---|---|
| global 経路未受信 / 空 | **publish しない。** `path_follower` は `local_trajectory` の staleness でゼロ指令に落ちる |
| local map が `local_map_timeout` (既定 0.5 s) より古い | **段 1 の trajectory だけを出す** (回避なし)。診断に理由を出す |
| 全候補が衝突で捨てられる (段 2) | **`Trajectory2D` を空 (points 0 個) で publish する。** これが「局所回避が解を持たない」の明示的な通知になり、`navigator` の停止トリガ経由で再計画に繋がる。**無言で古い trajectory を流さない** (F-18) |
| tf 失敗 | その周期をスキップ |

**主要パラメータ:** `update_frequency` / `lookahead_dist` / `horizon_time` / `trim_tail_dist` /
`max_linear_vel` / `max_angular_vel` / `max_accel` / `max_decel` / `max_lateral_accel` /
`local_map_timeout` / `use_local_avoidance` (段 2 の on/off) /
DWA: `vel_samples` / `omega_samples` / `sim_time` / `sim_steps` / `clearance_steps` /
`w_path` / `w_heading` / `w_velocity` / `w_clearance`

### 6.5 `path_follower` (`eltanin_controller`)

| 項目 | 内容 |
|---|---|
| 責務 | Local trajectory を追従し、要求速度を出す |
| 購読 | `local_path_planner/local_trajectory`、tf |
| 公開 | `~/cmd_vel_raw` (`TwistStamped`)、`~/lookahead_point` (`PointStamped`)、`~/follower_state` (`eltanin_msgs/NavigationState` の一部を再利用するか診断で出す) |
| 周期 | `update_frequency` 既定 20.0 Hz (NF-1) |
| 使う eltanin API | `control::PurePursuit`(+ E-3 の速度消費オーバーロード)、`control::GoalApproach` (E-10) |

**設計上の要点:**

- **`dt` は実経過時間** (C-8)。前回コールバック時刻との差分を使う。0 以下ならスキップ。
- **新しい trajectory を受け取ったときのリセット判断 (N-12 / §2.3):**
  `Trajectory2D` の `header.stamp` ではなく、**`navigator` が付与する `trajectory_generation` を
  区別する必要がある。** 実装は次のとおり。
  - `local_path_planner` は `header.frame_id` の後に続く連番を持たない。代わりに
    **`navigator` が `~/reset` (`std_srvs/Trigger`) を呼ぶ**ことでリセットする。
  - `navigator` は **停止トリガによる再計画のときだけ `~/reset` を呼ぶ**。走行中の観測トリガでは
    呼ばない (`reset()` は速度ランプを 0 に落として不要な減速を作る、§2.3)。
  - **この区別を「follower が自分で判断する」形にしない。** 判断材料 (なぜ再計画したか) は
    `navigator` にしかない。
- **`Status::NoPath` / `GoalReached` でも必ずゼロ指令を publish する** (F-6)。
  `eltanin` 側が保証しているものを `return` で潰さない。
- **向き合わせフェーズに進捗検査を持つ** (N-11)。`yaw_align_timeout` (既定 5.0 s) を超えたら
  `follower_state` に失敗を出し、ゼロ指令に落ちる。`navigator` が停止トリガとして拾う。

**失敗時の挙動:**

| 失敗 | 挙動 |
|---|---|
| trajectory 未受信 / `trajectory_timeout` (0.5 s) 超過 / 空 | **ゼロ指令を publish する。** publish を止めない |
| tf 失敗 | **ゼロ指令を publish する。** navyu は「何も publish せず return」だった (N-1) |

**主要パラメータ:** `update_frequency` / `desired_linear_vel` / `max_angular_vel` / `yaw_tolerance` /
`lookahead_time` / `min_lookahead_dist` / `trajectory_timeout` / `yaw_align_timeout` /
`xy_goal_tolerance` (0.10) / `yaw_goal_tolerance` (0.10) / `approach_distance` / `approach_decel`

### 6.6 `collision_predictor` (`eltanin_controller`)

**`/cmd_vel` の単一の所有者。安全に直結するので独立させる (F-7 / F-8)。**

| 項目 | 内容 |
|---|---|
| 責務 | 要求指令を local map に対して予測・制限し、**常に何かを出力する** |
| 購読 | `path_follower/cmd_vel_raw`、`local_map/local_map`、tf |
| 公開 | `/cmd_vel` (`geometry_msgs/Twist`)、`~/predicted_poses` (`nav_msgs/Path`)、`~/footprint` (`PolygonStamped`)、`~/diagnostics` |
| サービス | `~/enable_output` (`std_srvs/SetBool`) — **既定 `false` (D-22)** |
| 周期 | `update_frequency` 既定 20.0 Hz。**kachaka の watchdog 0.3 s に対し ≥4 Hz が必須** (C-15) |
| 使う eltanin API | `collision::VelocityLimiter::limit()` + **E-1 の厳密判定入口** |

**ウォッチドッグ (F-8 / N-1):**

```
毎周期:
  if (!output_enabled_)                       → publish しない (D-22)
  else if (いずれかの入力が期限切れ)          → ゼロ指令を publish
  else if (tf lookup 失敗)                    → ゼロ指令を publish
  else                                        → limit() の結果を publish
```

期限切れの対象は `cmd_vel_raw` (`cmd_timeout` 既定 0.3 s) と `local_map` (`map_timeout` 既定 0.5 s)。
**「古い指令を再送する」「何も publish しない」経路をコードに持たない。**

**入力を購読状態に書き戻さない (N-2):** 購読コールバックは `cmd_vel_raw_` に **受信値をそのまま**
書き、timer は**そのコピーを取って** `limit()` に渡す。制限値を `cmd_vel_raw_` に書き戻す経路が
存在しないことを、レビューで確認できる形 (`const` なローカル変数) に保つ。

**E-1 の厳密判定 (R-10 / 最優先):** `local_map` は膨張を持たないため、
`check_footprint()` の一段目ゲートが `Free` で短絡し、**フットプリントが `LETHAL` セルに重なる姿勢を
`Free` と返す。** `VelocityLimiterParams::exact_footprint_check = true` (E-1) を必ず設定する。
**このパラメータの既定値は `true` にする** — 忘れると安全が退化する方向なので、
明示的に `false` にしたときだけ二段構えになる形にする。

**主要パラメータ:** `update_frequency` / `prediction_steps` (10) / `prediction_time` (2.0) /
`collision_margin` (0.2) / `max_deceleration` / `footprint` (機体プロファイル) /
`exact_footprint_check` (true) / `cmd_timeout` / `map_timeout` / `output_enabled_on_startup` (sim: true / 実機: false)

### 6.7 `navigator` (`eltanin_navigator`)

| 項目 | 内容 |
|---|---|
| 責務 | ゴールを受け、`eltanin` の `NavigationSupervisor` (E-9) を駆動し、その `Action` を ROS の振る舞いに翻訳する |
| Action サーバ | `/navigate_to_pose` (`eltanin_msgs/NavigateToPose`) |
| Action クライアント | `global_path_planner/compute_path_to_pose` |
| サービスクライアント | `global_costmap/update`、`local_path_planner/enable`、`path_follower/reset` |
| 購読 | `local_map/local_map` (観測トリガの入力)、`collision_predictor/cmd_vel` (停止トリガの入力)、`global_path_planner/global_path`、tf |
| 公開 | `~/navigation_state` (`eltanin_msgs/NavigationState`) |
| 周期 | `update_frequency` 既定 20.0 Hz |

**状態機械 (D-26 / Q-15):**

| 状態 | 遷移先 | 契機 |
|---|---|---|
| `Idle` | `Planning` | Action goal 受理 |
| `Planning` | `Following` / `Failed` | `ComputePathToPose` の result |
| `Following` | `Replanning` / `Succeeded` / `Failed` / `Canceling` | `Supervisor::update()` の返す `Action` |
| `Replanning` | `Following` / `Failed` | `~/update` → `ComputePathToPose` の result |
| `Canceling` | `Idle` | `local_path_planner` を disable し、指令が 0 になるのを確認 |
| `Failed` / `Succeeded` | `Idle` | Action result を返した後 |

**判断そのものは `eltanin` に委ねる (D-10 / E-9 / F-19):**

```cpp
// navigator が毎周期やること (擬似コード)
const auto action = supervisor_.update({
  .pose = robot_pose,            // tf から
  .limited_command = last_cmd,   // /cmd_vel から
  .path = current_path,
  .observations = new_lethal_cell_centres,   // local_map の差分から
  .arc_length_since_replan = travelled,
  .follower_reached = follower_reported_goal_reached,
});
switch (action.kind) {
  case Continue: break;
  case Replan:   // 停止トリガなら path_follower/reset も呼ぶ
                 call_update_then_compute_path(action.reason);
  case Reached:  succeed_goal();
  case Fail:     abort_goal(action.fail_reason);
}
```

**閾値や判定式を `eltanin_ros` 側に持たない** (F-19)。`navigator` が持つのは
Action のライフサイクル、タイムアウト、誰をいつ呼ぶか、状態の publish だけである。

**`collision_predictor` を内側に置かない (F-19):** Action が無い / 完了した / キャンセルされた
状態でも `collision_predictor` の周期とウォッチドッグは動き続ける。`navigator` が
`collision_predictor` に対して行える操作は `~/enable_output` だけで、これも通常は bringup が
1 回呼ぶだけである。

**主要パラメータ:** `update_frequency` / `plan_timeout` / `costmap_update_timeout` /
Supervisor 系: `path_check_distance` (4.0) / `stop_cycles_to_replan` (5) / `stall_min_progress` (0.20) /
`max_replans` (3) / `xy_goal_tolerance` (0.10) / `yaw_goal_tolerance`

### 6.8 `simple_simulator` (`eltanin_simulator`)

| 項目 | 内容 |
|---|---|
| 責務 | 差動二輪 plant を ROS ノードとして提供する |
| 購読 | `/cmd_vel` (`Twist`)、`/initialpose` (`PoseWithCovarianceStamped`) |
| 公開 | `nav_msgs/Odometry`、tf (**`map → odom` と `odom → base_frame` の 2 段、F-11**)、`/scan` (合成 LiDAR、任意) |
| 周期 | `update_frequency` 既定 50 Hz |

- **`odom` フレームを出す** (§2.4-5 / F-11 / Q-2)。navyu は `map → base_footprint` を直接
  broadcast していたため AMCL が原理的に使えなかった。`map → odom` は真値 (誤差なし) を出し、
  外部 localization に差し替えたいときは launch 引数でこの broadcast を止められるようにする。
- **`nav_msgs/Odometry` を実際に publish する** (navyu は宣言のみ、N-13)。
- **`/initialpose` を実際に購読する** (navyu は宣言のみ、N-13)。
- **`cmd_vel` のタイムアウト** (`cmd_timeout` 既定 0.5 s) を持ち、超過したら停止する (F-11)。
- 合成 LiDAR は `navigation_loop.hpp` の `cast_scan` 相当を移植する。**`eltanin` には入れない**
  (デモ専用 = E-9 群 A に分類済み)。`eltanin_simulator` 内に置く。
  静的地図 + `~/obstacles` (`MarkerArray` か簡易な矩形パラメータ) から ground truth を作る。

**Gazebo は移植しない** (§3.2-5 / Q-9)。M3 / M4 の受け入れ条件は簡易シミュレータで満たせる
(`eltanin` の統合デモが同じ plant で全系を検証している)。

### 6.9 `eltanin_ros_common` (ライブラリのみ / ノードなし)

| ヘッダ | 内容 |
|---|---|
| `cost_conversion.hpp` | §5.3 の 4 関数。**値域変換はここだけに存在する** |
| `map_conversion.hpp` | `OccupancyGrid` ⇄ `eltanin::map::Costmap`、`eltanin_msgs/Costmap` ⇄ `Costmap`、origin yaw の検証 (C-2) |
| `geometry_conversion.hpp` | 四元数 ⇄ yaw、`Pose` ⇄ `Pose2D`、`Twist` ⇄ `Twist2D`、`TransformStamped` → `Transform2D` |
| `scan_conversion.hpp` | `LaserScan` → `ScanData` (コピー、N-4) |
| `path_conversion.hpp` | `eltanin::Path` ⇄ `nav_msgs/Path`、`Trajectory` ⇄ `eltanin_msgs/Trajectory2D` |
| `outcome_conversion.hpp` | `NavigateOutcome` ⇄ `eltanin_msgs` の `outcome` 定数 |
| `robot_profile.hpp` | フットプリント / 半径 / 速度上限 / 膨張パラメータの宣言・取得・検証を 1 箇所で行う |
| `stale_input.hpp` | `StaleInput<T>` — 値 + 最終更新時刻 + 期限。`get(now)` が期限切れなら `nullopt` を返す |
| `timing.hpp` | `PeriodicClock` — 実経過 `dt` の計算。`use_sim_time` に追従 (NF-4) |

**`StaleInput<T>` を型として持つ理由:** N-1 / N-4 は「期限を検査し忘れる」ことで起きた欠陥である。
**値を取り出す唯一の経路が期限検査を通る形**にすれば、検査を忘れることが書けなくなる。

---

## 7. パラメータと launch の構成 (F-14 / N-6)

**同じ値を 2 箇所に書かない。** navyu は 4 重複しノード名キーも値も乖離していた。

```
eltanin_bringup/config/
├── robot/kachaka.yaml          機体プロファイル (/** ワイルドカードで全ノードに配る)
├── robot/sim_robot.yaml        同上 (シミュレータ機体)
├── navigation.yaml             ノード固有パラメータ (ノード名キー)
└── rviz/eltanin.rviz
```

`robot/*.yaml` は ROS 2 のワイルドカードを使い、**1 箇所に書いた値を全ノードが同じキーで読む**。

```yaml
# robot/kachaka.yaml
/**:
  ros__parameters:
    robot:
      footprint: [-0.150, -0.120, 0.237, -0.120, 0.237, 0.120, -0.150, 0.120]
      inflation_radius: 0.55
      cost_scaling_factor: 10.0
      max_linear_vel: 0.30       # kachaka のブリッジ上限 (C-15)
      max_angular_vel: 1.57
      max_accel: 0.5             # 実機で計測して更新する (R-9)
      max_decel: 0.5
    frames:
      map: map
      odom: odom
      base: base_footprint
```

**フットプリントは kachaka の衝突ボックスから作る** (§2.6)。`base_link` 系で
`x ∈ [-0.150, 0.237]`, `y ∈ [-0.120, 0.120]` の矩形 (inscribed 0.120 / circumscribed 0.2656)。
**navyu の 0.6 m 角も `eltanin` の既定 0.6 m 角も使わない。**

**すべてのパラメータに意味のある既定値を与える** (F-14)。navyu はデフォルト無し宣言のため
キー不一致が起動時クラッシュになっていた。**既定値だけで起動できることを M1 の受け入れ条件にする。**

**launch 構成:**

| launch | 内容 |
|---|---|
| `eltanin_bringup.launch.py` | 全ノードを `ComposableNodeContainer` に載せる。引数: `robot_profile` / `use_sim_time` / `use_composition` / `use_rviz` / `map` / `autostart_output` |
| `simulation.launch.py` | `simple_simulator` + `eltanin_bringup` (`robot_profile:=sim_robot`, `autostart_output:=true`) |
| `kachaka.launch.py` | `eltanin_bringup` (`robot_profile:=kachaka`, `autostart_output:=false`) + remap 一式 + auto-homing 無効化 |

**`localization` 引数の宣言漏れ (§2.4-4) を作らない。** すべての引数を `DeclareLaunchArgument` する。

### 7.1 kachaka 起動シーケンス (F-12 / R-4 / R-5 / D-22)

```
1. zenoh ルータを起動する (手順を README と launch のコメントに明示)
2. kachaka ブリッジ (自前ビルド Jazzy + zenoh) を起動する
3. /kachaka/auto_homing/set_enabled を false にする  (ExecuteProcess)
4. 実行中の /kachaka/kachaka_command/execute をキャンセルする
5. eltanin_bringup を autostart_output:=false で起動する
   → collision_predictor は publish しない = teleop が有効化されない = ドック上でも動かない
6. ★ ロボットがドック上にないことを人が確認する
7. ros2 service call /collision_predictor/enable_output std_srvs/srv/SetBool "{data: true}"
   で出力を有効化する
   → 以降 20 Hz でゼロ指令を含む指令が出続ける (teleop の 60 s 失効も、
      最初の 1 指令の欠落も、常時 publish が構造的に吸収する)
8. ゴールを与える
```

**ステップ 6 が人の判断を要求する唯一の箇所である。** これを自動化しない。
**「ドック上での teleop 有効化が前進を引き起こす」ことを README に警告として書く** (F-17 / R-5)。

**zenoh 起因の要件 (F-12):** 全ノードで `RMW_IMPLEMENTATION=rmw_zenoh_cpp` に統一する。
`ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY` を分離の手段として当てにしない。
**`/kachaka/mapping/map` の `transient_local` が届くことを実測で確認する** (C-17 との相互作用)。

---

## 8. `eltanin` 側の変更 (ファイル単位)

**D-10 の帰結として残作業の大半が `eltanin` 側になる** (R-15)。以下がその全量である。
**すべて ROS を知らない形で定義できる** (R-14)。

### 8.1 新規ファイル

| ファイル | 内容 | 対応 |
|---|---|---|
| `include/eltanin/map/layers/raytrace_layer.hpp`<br>`src/map/layers/raytrace_layer.cpp` | clearing (`FREE_SPACE`) と marking (`LETHAL_OBSTACLE`) を合成優先規則つきで書くレイヤ | E-4 / E-5 |
| `include/eltanin/sensor/scan_clearing.hpp` (または `scan_projection.hpp` に追記)<br>`src/sensor/scan_clearing.cpp` | clearing 用の投影。`inf` ビームを `range_max` に置換し、**マーキングと同じ `ScanFilter` の角度範囲を要求する** | E-4 |
| `include/eltanin/core/trajectory.hpp`<br>`src/core/trajectory.cpp` | `TrajectoryPoint` (`Pose2D` + `linear_velocity` + `angular_velocity` + `time_from_start`) と `Trajectory` | E-2 |
| `include/eltanin/planner/velocity_profile.hpp`<br>`src/planner/velocity_profile.cpp` | `extract_window()` と `assign_velocity_profile()` (曲率 / 加減速 / ゴール減速の `min` 合成) | E-7 段 1 |
| `include/eltanin/planner/dwa_planner.hpp`<br>`src/planner/dwa_planner.cpp` | `DwaPlanner::create()` / `compute()`。判定モデルはテンプレート (既存の縫い目と同じ形) | E-7 段 2 |
| `include/eltanin/control/goal_approach.hpp`<br>`src/control/goal_approach.cpp` | ゴール接近の減速則と最終 yaw 合わせ。進捗検査つき | E-10 |
| `include/eltanin/navigation/supervisor.hpp`<br>`src/navigation/supervisor.cpp` | `NavigationSupervisor` — 停止カウンタ / ストール検出 / 再計画上限 / 到達判定 / leg 管理 | E-9 群 C |
| `include/eltanin/navigation/path_checks.hpp`<br>`src/navigation/path_checks.cpp` | `path_blocked_ahead()` / `lateral_error()` | E-9 群 B |
| `src/navigation/CMakeLists.txt` | 新モジュール `eltanin_navigation` (`eltanin::navigation`) | E-9 |
| `test/map/layers/test_raytrace_layer.cpp` | 合成優先規則 (marking 優先 / static 保護 / unknown 上書き) | E-5 |
| `test/sensor/test_scan_clearing.cpp` | `inf` ビームの置換 / 角度範囲の一致 / レンジ上限 | E-4 |
| `test/core/test_trajectory.cpp` | 型の基本操作 | E-2 |
| `test/planner/test_velocity_profile.cpp` | 曲率制限 / 加減速制限 / ゴール減速 / 退化ケース | E-7 |
| `test/planner/test_dwa_planner.cpp` | 衝突候補の除外 / 余裕評価 / 全候補衝突時の空出力 | E-7 |
| `test/control/test_goal_approach.cpp` | 減速則 / yaw 収束 / 進捗検査 | E-10 |
| `test/navigation/test_supervisor.cpp` | 停止 5 周期 / ストール (弧長) / 再計画上限 / 到達判定 / 観測トリガ | E-9 |
| `test/navigation/test_path_checks.cpp` | `path_blocked_ahead` / `lateral_error` (デモから移す既存の検証) | E-9 |

### 8.2 既存ファイルの変更

| ファイル | 変更 | 対応 | 破壊的か |
|---|---|---|---|
| `include/eltanin/collision/collision_checker.hpp` | `check_footprint_exact()` を追加 (一段目ゲートを通さず `contains_occupied_cell()` を直接呼ぶ。`OutsideMap` の判定は維持) | **E-1** | 非破壊 (追加のみ) |
| `include/eltanin/collision/velocity_limiter.hpp` | `VelocityLimiterParams::exact_footprint_check` (既定 **`true`**) を追加し、`limit()` の分岐に反映 | **E-1** | 既定値が変わるため**挙動が変わる**。既存テストの期待値を確認する |
| `include/eltanin/map/map_geometry.hpp` | `snapped_origin_for(window_cells, robot, clamped)` を追加 (`integration-design.md` §5 が候補として挙げている) | E-9 群 B / C-12 | 非破壊 |
| `include/eltanin/control/pure_pursuit.hpp`<br>`src/control/pure_pursuit.cpp` | `compute(robot, const Trajectory &, dt)` オーバーロードを追加。lookahead 点の目標速度をランプの目標にする。`Status` に `Approaching` を追加 | **E-3 / E-10** | `Status` への追加は `switch` の網羅性警告を出しうる。オーバーロードは非破壊 |
| `include/eltanin/map/layers/layer.hpp` ほかレイヤ 4 種 + `layered_costmap.{hpp,cpp}` | `update_bounds()` / `update_costs(master, bounds)` の 2 段化 | **E-6** | **破壊的。326 件超のテストに波及 (R-13)。実装順序の最後に置く (D-19)** |
| `CMakeLists.txt` | `add_subdirectory(src/navigation)` を追加 | E-9 | 非破壊 |
| `docs/costmap-design.md` / `docs/sensor-design.md` / `docs/control-design.md` / `docs/integration-design.md` | 各変更の設計判断を追記 (`eltanin` の慣行) | 全般 | — |
| `examples/navigation_loop.hpp` | 群 B / 群 C を `eltanin::navigation` の呼び出しに置き換える。群 A (デモ専用) は残す | E-9 | 統合テストが同じ数値を再現することで移行を検証する |

### 8.3 `eltanin` に**入れない**もの (境界の確認)

| 項目 | 置き場所 | 根拠 (D-10) |
|---|---|---|
| tf lookup とその失敗処理 | `eltanin_ros` | 「取得失敗が失敗経路になるか」 |
| スキャンの staleness / `cmd_vel` ウォッチドッグ | `eltanin_ros` | 「時計を必要とするか」 |
| フレーム名の保持・検証 | `eltanin_ros` | 「フレームを知る必要があるか」 |
| Action のライフサイクル / preempt / 状態の publish | `eltanin_ros` | 「他プロセスを順序づけるか」 |
| 合成 LiDAR (`cast_scan`) / 偽障害物の注入 / ground truth 照合 | `eltanin_simulator` / 残置 | E-9 群 A (デモ専用) |
| コスト値域の ROS 変換 | `eltanin_ros_common` | ROS の値域は ROS 側の関心 |
| localization | **どちらにも入れない** | D-8 / §3.2-1 |

---

## 9. `eltanin_ros` 側の新規ファイル (パッケージ単位)

```
eltanin_msgs/            msg 6 / action 2 (§5.1。srv は作らない)
eltanin_vendor/          CMakeLists.txt (ExternalProject), package.xml,
                         test/link_check/{CMakeLists.txt,link_check.cpp}  ← PIC / 消費経路の検証
eltanin_ros_common/      include/eltanin_ros_common/*.hpp (9 本, §6.9), src/*.cpp,
                         test/test_cost_conversion.cpp, test_map_conversion.cpp,
                         test_geometry_conversion.cpp, test_scan_conversion.cpp,
                         test_path_conversion.cpp, test_robot_profile.cpp,
                         test_stale_input.cpp, test_exact_footprint_regression.cpp  ← M1 の E-1 回帰
eltanin_costmap/         src/local_map_node.cpp, src/global_costmap_node.cpp,
                         include/.../{local_map,global_costmap}.hpp,
                         test/test_window_snap.cpp        ← C-12 の 1:1 再標本化
                         test/test_local_map_composition.cpp  ← E-5 の優先規則
                         test/test_update_region.cpp      ← E-6 の off-by-r
eltanin_planner/         src/global_path_planner_node.cpp, src/local_path_planner_node.cpp,
                         include/..., test/test_goal_validation.cpp
eltanin_controller/      src/path_follower_node.cpp, src/collision_predictor_node.cpp,
                         include/..., test/test_watchdog.cpp   ← N-1 / N-2 の回帰
eltanin_navigator/       src/navigator_node.cpp, include/..., test/test_state_machine.cpp
eltanin_simulator/       src/simple_simulator_node.cpp, src/synthetic_scan.cpp, include/...
eltanin_bringup/         launch/ 3 本, config/ (§7), rviz/eltanin.rviz, map/ (navyu の地図を参照)
eltanin_ros/             package.xml のみ
リポジトリ直下           README.md, .clang-format (navyu 準拠), .pre-commit-config.yaml,
                         .github/workflows/{colcon-build.yml,pre-commit.yaml}, docs/
```

**`navyu` から移植しないもの (N-13 / F-16):** `broadcaster_` / `predict_path_publisher_` /
`get_transform()` / `use_radius_foot_print` / `foot_print_radius` / `covariance_` /
構築されない `initial_pose_subscriber_` と `odometry_publisher_` / 存在しない
`global_costmap_params.yaml` への参照。**RViz が表示しているのに誰も publish していないトピックを
作らない** (F-13)。

**`package.xml` の規約 (N-10 / F-16):** ライブラリターゲット名を依存パッケージとして宣言しない。
宣言が実際の `#include` と一致することを CI で確認する
(`rosdep install -y --from-paths src --ignore-src` の成功を M1 の受け入れ条件にする)。

---

## 10. 実装順序

**分割の基準は「そのステージだけで `colcon build` と `colcon test` が通り、前のステージの
受け入れ条件を壊さないこと」である。** 破壊的変更 (E-6) を最後に置くのがこの順序の要点。

| # | ステージ | 内容 | 完了条件 | M |
|---|---|---|---|---|
| **S0** | リポジトリ基盤 | `.clang-format` (navyu 準拠) / `.pre-commit-config.yaml` / CI 2 本 / `README.md` の骨格 / 本設計文書 / `eltanin_ros` メタパッケージ。**`main` を作る (D-28)** | `pre-commit` が通る | — |
| **S1** | **E-1 (eltanin)** | `check_footprint_exact()` + `exact_footprint_check` (既定 true) + 回帰テスト。**既存の `VelocityLimiter` テストの期待値を確認する** | 「中心セルが `FREE_SPACE`、フットプリントが `LETHAL` に重なる姿勢で `Collision`」が固定される (R-10) | M1 |
| **S2** | vendor + msgs + 変換層 | `eltanin_vendor` / `eltanin_msgs` / `eltanin_ros_common` + 単体テスト | `rosdep install` 成功 / `colcon build` 警告なし / 往復整合と origin yaw 拒否と E-1 回帰が固定される | **M1** |
| **S3** | global 経路まで | `global_costmap` + `global_path_planner` + `eltanin_bringup` (最小) + RViz | ゴールを与えて経路が出る / 到達不能ゴールで明示的な失敗が観測できる / `frame_id` 不一致を検出する | **M2** |
| **S4** | **E-10 (eltanin)** | `GoalApproach` + `PurePursuit::Status::Approaching` + テスト | 減速則と yaw 収束と進捗検査が固定される | M3 |
| **S5** | 閉ループ | `path_follower` + `collision_predictor` + `simple_simulator` + `navigator` **最小形** (Planning → Following → Succeeded / Failed のみ) | 走って到達する / TF 断でゼロ指令 / `cmd_vel` 停止で 0 にラチェットしない / 新ゴールで状態リセット / `use_sim_time` が効く | **M3** |
| **S6** | **E-4 / E-5 (eltanin)** | clearing 投影 + `RaytraceLayer` + テスト | 3 値の区別と marking 優先が固定される | M4 |
| **S7** | local map | `local_map` ノード + 窓スナップのテスト | 観測が反映される / 窓が格子にスナップし再標本化が 1:1 / スキャン停止で期限切れになる | M4 |
| **S8** | **E-2 / E-3 (eltanin)** | `Trajectory` 型 + `PurePursuit` の速度消費オーバーロード + `assign_velocity_profile()` | 速度プロファイルが follower に反映される | M4 |
| **S9** | local planner 段 1 | `local_path_planner` (窓切り出し + 速度付与、回避なし) | 閉ループが trajectory 経由で成立し、S5 の受け入れ条件を維持する | M4 |
| **S10** | **E-7 (eltanin)** + 段 2 | `DwaPlanner` + `local_path_planner` に組み込み | 局所回避が動作する / 全候補衝突時に空 trajectory を出す | M4 |
| **S11** | **E-9 (eltanin)** + 再計画 | `NavigationSupervisor` + `path_checks` + `navigator` の Replanning 状態 | 未知障害物で減速・停止・再計画・迂回して到達する / ストールで有限時間に失敗する / 再計画上限で失敗する / 全周期無衝突 | **M4** |
| **S12** | kachaka 実機 | `kachaka.launch.py` + 機体プロファイル + 起動シーケンス + README の安全警告 | §7 の M5 受け入れ条件 | **M5** |
| **S13** | **E-6 (eltanin)** | `Layer` の ROI 2 段化 + `map_updates` の完全形 | 差分が ⊕`r` で publish され窓端の膨張が欠けない / 既存 326 件超が通る | 後 |

### 10.1 この順序の根拠

- **S1 が最初なのは安全に直結するから** (R-10)。非膨張マップに対する判定の退化は
  「実機で衝突する」欠陥であり、これを抱えたまま先へ進まない。
- **S13 (E-6) が最後なのは唯一の破壊的変更だから** (D-19 / R-13)。
  `Layer` インタフェースの変更は 326 件超のテストに波及する。
  **S3 〜 S12 は E-6 なしで成立する** — global の更新契機を再計画時に限り (C-3 の回避)、
  差分矩形をノード側で追跡する (§6.2) 設計にしたためである。
- **eltanin の変更 (S1 / S4 / S6 / S8 / S10 / S11 / S13) が `eltanin_ros` のステージと交互に来る。**
  並行開発 (A-5) では、`eltanin` 側のステージを先に完了させてから `eltanin_ros` 側に進む。
  `eltanin_vendor` が `BUILD_ALWAYS ON` で隣接ソースを追うので、`eltanin` の変更は
  次の `colcon build` で反映される (D-24)。
- **S5 の `navigator` を最小形にする理由**: 再計画 (S11) は local map (S7) と Supervisor (S11) を
  必要とする。M3 の時点では「計画 → 追従 → 到達 / 失敗」だけで閉ループが成立し、
  N-1 / N-2 / N-12 の回帰をここで固定できる。
- **S9 (段 1) と S10 (段 2) を分ける理由** (D-16 / R-12): 段 1 だけで
  「trajectory 型 + 速度の受け渡し + follower の速度消費」が検証でき、
  DWA のデバッグをそれらの疑いから切り離せる。

### 10.2 検証の基準値 (A-2)

`eltanin` の実測値を回帰の基準に使う。**`eltanin_ros` 側で数値を作り直さない。**

| 項目 | 基準 | 出典 |
|---|---|---|
| 経路からの最大逸脱 | 0.052 m (dt 0.05、クリーン地図) | `integration-design.md` §12 |
| 最終位置誤差 | 0.024 m (クリーン) / 0.011 m (障害物あり) | 同 |
| 再計画回数 | 1 (障害物あり、観測トリガ) | 同 |
| 走行中の最小フットプリント余裕 | 0.104 m | 同 §11 |
| 衝突した通過姿勢 | **0** | 同 §12 |
| 停止余裕 | `stop_clearance` 0.25 m / `stop_obstacle_clearance` 0.296 m | 同 §11 |

シミュレータの plant と dt が同じであれば、**S11 の完了時点でこれらに近い値が出るはずである。**
大きく外れたら ROS 層 (時刻・座標・周期) に原因がある。

---

## 11. リスクと未確定事項

### 11.1 要件定義から状態が変わったリスク

| # | 状態 |
|---|---|
| R-2 (C-3 を回避しきれない) | **閉じた。** global の更新契機を `~/update` サービスに限り、差分矩形をノード側で追跡する (§6.2 / D-19) |
| R-10 (E-1 の見落とし) | **緩和。** `exact_footprint_check` の既定を `true` にし (§6.6)、S1 を最初のステージにした |
| R-11 (膨張が 2 ホップ先) | **閉じた。** D-15 により local planner は膨張コストを使わず、非膨張 local map への厳密判定で評価する |
| R-13 (E-6 の波及) | **緩和。** S13 に隔離し、S3 〜 S12 が E-6 に依存しない設計にした (D-19) |
| R-12 (E-7 の工数) | **緩和。** D-16 の 2 段導入で S9 / S10 に分割した |
| R-7 (ゴール振動) | **緩和。** D-17 (E-10) で S4 に前倒しした |
| R-9 (加減速限界が不明) | **残る。** `max_accel` / `max_decel` は S12 の前に実機で計測する。それまでは保守的な 0.5 を使う |
| R-3 / R-4 / R-5 / R-6 / R-8 / R-14 / R-15 | **要件定義のまま。** 緩和策は §7.1 / §8.3 / §10.1 に織り込んだ |

### 11.2 この設計で新たに生じるリスク

| # | リスク | 影響 | 緩和 |
|---|---|---|---|
| **R-16** | **DWA の余裕評価 (段階的フットプリント膨張) が横方向余裕を十分に確保できない** | 障害物すれすれを通る。`integration-design.md` §15.2 の 0.111 m がさらに縮む可能性 | S10 で走行全体の最小フットプリント余裕を計測し、`eltanin` の 0.104 m と比較する。不足すれば距離場 (E-8) を導入する (U-1) |
| **R-17** | **`local_map` が `StaticLayer` を持つため、自己位置誤差が「静的壁の位置ずれ」として local map に入る** | `collision_predictor` が実在しない壁で止まる / 実在する壁を見落とす | kachaka は `map → odom` をロボット自身が出すため誤差は小さいと期待できるが、S12 で実測する。`clear_static_obstacles` を true にする判断はここで行う |
| **R-18** | **`RelWithDebInfo` で `assert` が無効になるため、`GridMap` の範囲外アクセスが UB になる** | 実機で無言のメモリ破壊 | §4.3 の規約 (境界検査付き API のみを使う) を CI の grep チェックで機械的に確認する。CI は Debug ビルドも回す |
| **R-19** | **intra-process comms で `eltanin_msgs/Costmap` を共有したとき、購読側が `data` を書き換えると他の購読者を壊す** | N-4 と同種の欠陥を新しい場所で作る | 購読コールバックは `const` 参照で受け、`eltanin::map::Costmap` へ**コピー**して抜ける。`ConstSharedPtr` 購読を規約とする |
| **R-20** | **`navigator` が `path_follower/reset` を呼ぶタイミングで、follower が既に次の周期を回している** | 不要な減速、または reset が効かない | `reset` は `Trigger` サービスで、follower 側は次の `compute()` の前に適用する。競合は callback group と mutex で閉じる (§3.3) |

### 11.3 未確定事項 (いずれも M4 以降に判断を遅らせられる)

| # | 未確定 | 判断の時期と材料 |
|---|---|---|
| **U-1** | **距離場 (E-8) を導入するか** | **S10 の完了時点。** 走行全体の最小フットプリント余裕が `eltanin` の 0.104 m を大きく下回るなら導入する。導入する場合は `local_map` が local 窓の距離場を併せて出す (D-23)。global 側は静的なので 1 回計算すれば済む |
| **U-2** | **`clear_static_obstacles` を true にするか** | **S12 の実機走行時。** 静的地図に無い動的障害物は既定 (false) でも消せる。地図が古くて実在しない壁が残る場合にのみ true が必要になる。true にすると自己位置誤差で実在する壁を消す危険が生じる (R-17) |
| **U-3** | **スキャンの角度セクタ除外が必要か** (Q-16-1) | **S12 の実機走行時。** kachaka のスキャンは全周で、既定は `nullopt`。棚を積載した場合に棚を見るビームを落とす必要があるなら、**1 つの `AngleRange` では中央セクタの除外を表現できない** ため `ScanFilter` の拡張 (`eltanin` 変更) になる |
| **U-4** | **外部 localization (emcl2 等) に切り替えるか** (D-8 / Q-2) | **S12 の実機走行時。** kachaka の自己位置推定で足りなければ切り替える。`simple_simulator` が `odom` を出す設計 (§6.8) にしたことで、**シミュレータでも外部 localization が原理的に使える** ようになっており、切り替えは launch の差分だけで済む |

---

## 12. 参照

### `eltanin`

- `AGENTS.md` — 依存規則の絶対制約 (C-14)
- `docs/costmap-design.md` — §8.1.1 (コスト値の一致を目指さない) / §14.1-14.7 (レイヤ境界と実測値) /
  **§15 (ROS ノード構成への申し送り = 本設計の出発点)**
- `docs/sensor-design.md` — §7 (tf を持たない理由) / **§11 (clearing が表現できない理由 = E-4 の根拠)**
- `docs/planner-design.md` — §6.1 (横方向マージン 1.62 cm) / §12 (基底クラスを作らなかった理由)
- `docs/control-design.md` — §1 (純関数化の契約) / §10 (navyu 欠陥対応表)
- `docs/collision-design.md` — §2 (二段構え衝突判定 = E-1 の根拠) / §9 (ROS ノード化への申し送り)
- `docs/integration-design.md` — §3 (1 周期の順序) / §5 (格子スナップ) / §9 (再計画・停止・ストール) /
  §12 (実測値 = §10.2 の基準) / §13 (既知の欠落) / **§16 (境界付き更新の申し送り = E-6)**
- `examples/navigation_loop.hpp` — 閉ループの仕様書。群 B / C を E-9 で `eltanin` へ移す

### `navyu`

- `navyu_navigation/launch/navyu_bringup.launch.py` / `config/navyu_params.yaml` — 置き換え対象
- `.clang-format` / `.pre-commit-config.yaml` / `.github/workflows/` — スタイルと CI の基準

### `kachaka-api`

- `ros2/kachaka_grpc_ros2_bridge/src/component/manual_control_component.cpp` — 速度指令の制約 (C-15)
- `ros2/kachaka_description/urdf/_kachaka.urdf.xacro` — 機体諸元 (§7 のフットプリント)
- `ros2/demos/kachaka_nav2_bringup/launch/navigation_launch.py` — remap の実例
