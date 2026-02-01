# URDF から RobotModel を生成するコードの解説

このドキュメントは、MoveIt2 のコードベースで **URDF/SRDF から `moveit::core::RobotModel` を生成する流れ** を追い、主要クラスと責務、生成手順を整理したものです。実際の実装は以下のファイルを中心に参照してください。

- `moveit_ros/planning/rdf_loader/src/rdf_loader.cpp`
- `moveit_ros/planning/robot_model_loader/src/robot_model_loader.cpp`
- `moveit_core/robot_model/src/robot_model.cpp`
- `moveit_core/planning_scene/src/planning_scene.cpp`

## 全体の流れ (概要)

1. **RDFLoader** が URDF/SRDF の文字列を取得・パースする
2. **RobotModelLoader** が RDFLoader を使って `moveit::core::RobotModel` を生成する
3. **RobotModel** が URDF/SRDF を元にリンク/ジョイント/グループを構築する
4. (任意) **PlanningScene** が `RobotModel` を受け取ってシーンを初期化する

```
URDF/SRDF (param or string)
        |
        v
rdf_loader::RDFLoader
        |
        v
robot_model_loader::RobotModelLoader
        |
        v
moveit::core::RobotModel
```

## 1. RDFLoader: URDF/SRDF の取得とパース

**ファイル**: `moveit_ros/planning/rdf_loader/src/rdf_loader.cpp` / `.../rdf_loader.h`

- `RDFLoader` は **URDF/SRDF の文字列を確保してパース** する責務を持ちます。
- 2つの使い方があります。
  - **ROS2 パラメータ / トピック** から取得する (`RDFLoader(node, "robot_description")`)
  - **文字列を直接渡す** (`RDFLoader(urdf_string, srdf_string)`)

### パラメータ/トピック経由の取得

- `robot_description` と `robot_description_semantic` (SRDF) を `SynchronizedStringParameter` 経由で取得します。
- パラメータが無ければトピック購読にフォールバックする設計です。
- 取得した文字列は `loadFromStrings()` でパースされます。

### 文字列からのパース

- `urdf::Model::initString()` で URDF をパース
- `srdf::Model::initString()` で SRDF をパース
- 両方成功した場合に `urdf_` と `srdf_` を保持します。

## 2. RobotModelLoader: RobotModel の生成と付随設定

**ファイル**: `moveit_ros/planning/robot_model_loader/src/robot_model_loader.cpp`

- `RobotModelLoader` は `RDFLoader` を内部で作成し、
  `moveit::core::RobotModel` を組み立てる入口となるクラスです。

### 生成の流れ

1. `RDFLoader` を生成し、URDF/SRDF をパース
2. `moveit::core::RobotModel` を作成
3. 追加の **joint_limits パラメータ** を反映
4. (オプション) **IK プラグインをロード**

### 追加の Joint Limits

`robot_description_planning.joint_limits.<joint_name>.*` のパラメータで
URDF の limit を上書きできます。

- `max_position`, `min_position`
- `has_velocity_limits`, `max_velocity`
- `has_acceleration_limits`, `max_acceleration`
- `has_jerk_limits`, `max_jerk`

URDF の内容だけでなく、**パラメータで限界値を補正できる**のがポイントです。

### IK プラグインのロード

- `kinematics_plugin_loader::KinematicsPluginLoader` を使って
  SRDF で定義されたグループに対し IK を割り当てます。
- `RobotModel::setKinematicsAllocators()` を使って関連付けが行われます。

## 3. RobotModel: URDF/SRDF から内部モデルを構築

**ファイル**: `moveit_core/robot_model/src/robot_model.cpp`

### buildModel() が中心

`RobotModel` のコンストラクタから `buildModel()` が呼ばれ、
以下の処理が順に実行されます。

1. **buildRecursive()**
   - URDF のツリーを深さ優先で走査して
     `JointModel` と `LinkModel` を生成
2. **buildMimic()**
   - URDF の mimic joint を反映
3. **buildJointInfo()**
   - インデックスや変数数、派生情報を整理
4. **buildGroups()**
   - SRDF の group 情報から `JointModelGroup` を生成
5. **buildGroupStates()**
   - SRDF の group_state を組み込む

### buildRecursive(): URDF ツリーの変換

- `constructJointModel()` でジョイントを構築
- `constructLinkModel()` でリンクを構築
- 子リンクに対して再帰処理を繰り返します

### constructJointModel() のポイント

- URDF の joint type に対応して `Revolute/Prismatic/Floating/...` を作成
- **ルートジョイントは SRDF の virtual joint** を参照
  - もし指定が無ければ `ASSUMED_FIXED_ROOT_JOINT` が作られる
- SRDF の `passive_joints` や `joint_properties` が反映される

### constructLinkModel() のポイント

- collision geometry を `LinkModel` に取り込む
- collision が無く visual だけある場合は警告
- visual mesh 情報も `LinkModel` に保持される

## 4. PlanningScene との関係

**ファイル**: `moveit_core/planning_scene/src/planning_scene.cpp`

- `PlanningScene(urdf_model, srdf_model)` でも `RobotModel` を生成できます。
- `createRobotModel()` を通じて
  `RobotModel` を生成し、ルートジョイントの有無を確認します。

## 5. どこを追えば良いか (コードナビ)

- **URDF/SRDF 文字列の取得**:
  `rdf_loader::RDFLoader` (`.../rdf_loader.cpp`)
- **RobotModel の入口**:
  `robot_model_loader::RobotModelLoader` (`.../robot_model_loader.cpp`)
- **内部構築の詳細**:
  `moveit::core::RobotModel` (`moveit_core/robot_model/src/robot_model.cpp`)
- **PlanningScene 側の生成**:
  `PlanningScene::createRobotModel()` (`moveit_core/planning_scene/src/planning_scene.cpp`)

## 補足: 最小経路での理解

「URDF から RobotModel を作る」だけを追うなら、
以下の順で読むと最短です。

1. `RobotModelLoader::configure()`
2. `RDFLoader::loadFromStrings()`
3. `RobotModel::buildModel()` → `buildRecursive()`
4. `RobotModel::constructJointModel()` / `constructLinkModel()`
