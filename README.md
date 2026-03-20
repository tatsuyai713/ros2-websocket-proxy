# ROS 2 WebSocket Proxy

> **[English](#english)** | **[日本語](#japanese)**

---

<a id="english"></a>

## English

### Overview

**ros2_websocket_proxy** is a lightweight, high-performance bridge that tunnels
arbitrary ROS 2 topics over a single WebSocket connection. It serializes
messages at the CDR level — the native ROS 2 wire format — so **no
message-type code generation or rebuilding is required on the proxy side**.
Simply list the topics and their types in a YAML config and the bridge is
ready.

A typical deployment connects two physically separate ROS 2 networks (e.g. a
robot and a monitoring station) over any TCP/IP link, including Wi-Fi, VPN, or
the public Internet.

Unlike standard ROS 2 discovery (DDS) or rosbridge, this proxy **forwards only
the topics you explicitly list** in the YAML config. In a system with hundreds
of active topics, the remote side sees *only* the topics it needs — no
unexpected topic pollution, no namespace confusion, and no unnecessary bandwidth
consumption.

```
┌─────────────────────┐          WebSocket          ┌──────────────────────┐
│   ROS 2 Network A   │◄══════════════════════════►│   ROS 2 Network B    │
│                      │                             │                      │
│  generic_server      │         TCP / Wi-Fi         │  generic_client      │
│  (server node)       │◄─────────────────────────►│  (client node)       │
│                      │                             │                      │
│  ┌──────────────┐    │                             │    ┌──────────────┐  │
│  │ ROS 2 Topics │◄──►│                             │◄──►│ ROS 2 Topics │  │
│  └──────────────┘    │                             │    └──────────────┘  │
└─────────────────────┘                             └──────────────────────┘
```

### Key Features

| Feature | Description |
|---------|-------------|
| **Type-agnostic** | Uses `GenericPublisher` / `GenericSubscription` with CDR serialization — works with *any* message type without recompilation. |
| **Low latency** | Zero-copy design where possible; a single `memcpy` per send/receive path. No intermediate `std::vector` allocations in the hot path. |
| **Bidirectional** | Each side can simultaneously subscribe to and publish topics through the same WebSocket connection. |
| **Auto-reconnect** | The client node automatically reconnects when the connection drops. |
| **YAML-driven** | Topic names and types are configured entirely through YAML files — no code changes needed to add or remove topics. |
| **Thread-safe** | Shared state is protected by `std::mutex` and `std::atomic`, preventing data races between the ROS executor and the WebSocket I/O thread. |
| **Lightweight** | Single header-only dependency (websocketpp + Asio). No rosbridge, no JSON overhead. |
| **Selective forwarding** | Only the topics listed in the YAML config are transferred. Unlike DDS discovery or rosbridge, no extra topics leak to the remote side — ideal for large systems with many topics. |
| **Cross-domain bridge** | Works as a bridge between different ROS 2 DDS domain IDs on the same machine. Run the server in one domain and the client in another to selectively share topics across domains without changing `ROS_DOMAIN_ID` globally. |

### Use Cases

#### 1. Selective Topic Forwarding Across Networks

In a typical ROS 2 multi-machine setup, DDS discovery exposes **every** topic
to **every** participant on the network. When a monitoring station joins the
same DDS domain as a robot running dozens of sensor drivers, all topics become
visible, which can:

- Confuse operators with irrelevant topics.
- Consume bandwidth with unwanted data.
- Leak internal implementation details.

With `ros2_websocket_proxy`, you explicitly choose which topics cross the
boundary. The remote side only sees what you intend to share.

```
Robot (many topics)              Monitoring Station
  /camera/image    ──────►         /camera/image
  /lidar/points    ──────►         /lidar/points
  /cmd_vel         ◄──────         /cmd_vel
  /joint_states    (blocked)
  /diagnostics     (blocked)
  /tf              (blocked)
```

#### 2. Cross-Domain Communication on the Same Machine

ROS 2 uses `ROS_DOMAIN_ID` to isolate DDS traffic. Nodes in different domains
cannot discover each other. This is useful for isolation, but sometimes you need
to share a *few* topics between domains — for example, a simulation in Domain 1
and a planner in Domain 2.

Run `generic_server` in Domain 1 and `generic_client` in Domain 2 (or vice
versa). Because WebSocket operates over TCP (localhost), it bypasses DDS domain
boundaries entirely:

```bash
# Terminal 1 — Domain 1 (e.g. simulation)
export ROS_DOMAIN_ID=1
ros2 launch ros2_websocket_proxy websocket_server.launch.py

# Terminal 2 — Domain 2 (e.g. planner)
export ROS_DOMAIN_ID=2
ros2 launch ros2_websocket_proxy websocket_client.launch.py \
    ws_url:=ws://localhost:9090
```

Only the topics listed in the YAML configs are bridged; all other topics remain
isolated within their respective domains.

### Wire Protocol

Every WebSocket binary frame has the following layout:

```
Offset   0                          128              128 + N
         ├──── Topic Name ──────────┤──── CDR Payload ────┤
         │  (null-padded, 128 B)    │   (N bytes)         │
```

- **Bytes 0–127**: The ROS topic name, null-padded to exactly 128 bytes.
- **Bytes 128–end**: The CDR-serialized ROS message, forwarded as-is.

### Comparison with rosbridge_suite

| Aspect | ros2_websocket_proxy | rosbridge_suite |
|--------|---------------------|-----------------|
| Serialization | CDR (native binary) | JSON |
| Overhead | 128 B header per frame | JSON envelope + Base64 for binary data |
| Latency | Minimal (single memcpy) | Higher (serialize → JSON → parse) |
| Bandwidth | Near-native | 2–4× for binary payloads (Base64 + JSON) |
| Topic visibility | Only configured topics | All topics visible after advertise |
| Configuration | YAML topic list | Runtime advertise / subscribe |
| Use case | Machine-to-machine, low-latency, topic isolation | Web browser integration |

### Requirements

- ROS 2 (Humble / Iron / Jazzy or later recommended)
- C++17 compiler (GCC 9+ or Clang 10+)
- [websocketpp](https://github.com/zaphoyd/websocketpp)
- [Asio](https://think-async.com/Asio/) (standalone, header-only)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

### Installation

#### Install Dependencies (Ubuntu)

```bash
cd ~/ros2_ws/src/ros2-websocket-proxy
bash scripts/install_dependency.sh
```

#### Build

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_websocket_proxy
source install/setup.bash
```

### Configuration

Create or edit YAML files in the `config/` directory.

**Server side** (`config/server_topics.yaml`):

```yaml
subscribe_topics:          # ROS → WebSocket (forwarded to client)
  - name: "/camera/image"
    type: "sensor_msgs/msg/Image"

publish_topics:            # WebSocket → ROS (received from client)
  - name: "/cmd_vel"
    type: "geometry_msgs/msg/Twist"
```

**Client side** (`config/client_topics.yaml`):

```yaml
subscribe_topics:          # ROS → WebSocket (forwarded to server)
  - name: "/cmd_vel"
    type: "geometry_msgs/msg/Twist"

publish_topics:            # WebSocket → ROS (received from server)
  - name: "/camera/image"
    type: "sensor_msgs/msg/Image"
```

> **Note** — `subscribe_topics` on one side should correspond to
> `publish_topics` on the other, and vice versa.

### Usage

#### Using Launch Files

```bash
# Server (Machine A)
ros2 launch ros2_websocket_proxy websocket_server.launch.py \
    yaml_file:=server_topics.yaml port:=9090

# Client (Machine B)
ros2 launch ros2_websocket_proxy websocket_client.launch.py \
    yaml_file:=client_topics.yaml ws_url:=ws://<SERVER_IP>:9090
```

#### Using ros2 run

```bash
# Server
ros2 run ros2_websocket_proxy generic_server \
    --ros-args -p yaml_file:=server_topics.yaml -p port:=9090

# Client
ros2 run ros2_websocket_proxy generic_client \
    --ros-args -p yaml_file:=client_topics.yaml -p ws_url:=ws://<SERVER_IP>:9090
```

### Parameters

#### generic_server

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `yaml_file` | string | `server_topics.yaml` | YAML config filename (resolved under `config/`). |
| `port` | int | `9090` | TCP port for the WebSocket server. |

#### generic_client

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `yaml_file` | string | `client_topics.yaml` | YAML config filename (resolved under `config/`). |
| `ws_url` | string | `ws://localhost:9090` | WebSocket server URL to connect to. |

### License

MIT License

---

<a id="japanese"></a>

## 日本語

### 概要

**ros2_websocket_proxy** は、任意の ROS 2 トピックを 1 本の WebSocket 接続で
トンネリングする軽量・高性能ブリッジです。CDR（ROS 2 ネイティブのバイナリ形式）
レベルでメッセージをシリアライズするため、**プロキシ側でのメッセージ型コード生成や
再ビルドは不要**です。YAML 設定ファイルにトピック名と型を列挙するだけで
ブリッジが動作します。

典型的なユースケースは、物理的に離れた 2 つの ROS 2 ネットワーク（例: ロボットと
モニタリングステーション）を Wi-Fi、VPN、インターネットなど任意の TCP/IP 経路で
接続する場面です。

ROS 2 標準の DDS ディスカバリや rosbridge とは異なり、このプロキシは **YAML に
明示的に記載したトピックのみ** を転送します。多数のトピックが流れるシステムでも、
リモート側には必要なトピックだけが見え、不要なトピックの混入・名前空間の混乱・
帯域の浪費を防ぎます。

```
┌─────────────────────┐         WebSocket           ┌──────────────────────┐
│  ROS 2 ネットワーク A │◄═══════════════════════════►│  ROS 2 ネットワーク B  │
│                      │                             │                      │
│  generic_server      │        TCP / Wi-Fi          │  generic_client      │
│  (サーバーノード)      │◄─────────────────────────►│  (クライアントノード)   │
│                      │                             │                      │
│  ┌──────────────┐    │                             │    ┌──────────────┐  │
│  │ ROS 2 トピック│◄──►│                             │◄──►│ ROS 2 トピック│  │
│  └──────────────┘    │                             │    └──────────────┘  │
└─────────────────────┘                             └──────────────────────┘
```

### 主な特徴

| 特徴 | 説明 |
|------|------|
| **型非依存** | `GenericPublisher` / `GenericSubscription` と CDR シリアライゼーションを使用。再コンパイルなしで *あらゆる* メッセージ型に対応。 |
| **低レイテンシ** | 送受信パスごとに `memcpy` 1 回のみ。中間的な `std::vector` アロケーションを排除。 |
| **双方向通信** | 1 本の WebSocket 接続で subscribe と publish を同時に実行可能。 |
| **自動再接続** | クライアントノードは接続断時に自動で再接続。 |
| **YAML 駆動** | トピック名と型は YAML ファイルで設定。トピックの追加・削除にコード変更不要。 |
| **スレッドセーフ** | `std::mutex` と `std::atomic` で共有状態を保護し、ROS エグゼキュータと WebSocket I/O スレッド間のデータ競合を防止。 |
| **軽量** | ヘッダオンリーの依存 (websocketpp + Asio) のみ。rosbridge 不要、JSON オーバーヘッドなし。 |
| **選択的転送** | YAML に記載したトピックのみ転送。DDS ディスカバリや rosbridge と異なり、不要なトピックがリモート側に漏れない。大規模システムに最適。 |
| **クロスドメインブリッジ** | 同一マシン上の異なる ROS 2 DDS ドメイン ID 間のブリッジとしても動作。`ROS_DOMAIN_ID` をグローバルに変更せず、選択的にトピックを共有可能。 |

### ユースケース

#### 1. ネットワーク越しの選択的トピック転送

一般的な ROS 2 マルチマシン構成では、DDS ディスカバリにより **すべての** トピックが
ネットワーク上の **すべての** 参加者に公開されます。例えば、多数のセンサードライバが
動作するロボットと同じ DDS ドメインにモニタリングステーションが参加すると、
全トピックが見えてしまい：

- オペレーターが無関係なトピックに混乱する
- 不要なデータで帯域を消費する
- 内部実装の詳細が外部に漏れる

`ros2_websocket_proxy` を使えば、境界を越えるトピックを明示的に選択でき、
リモート側には共有したいトピックだけが見えます。

```
ロボット (多数のトピック)           モニタリングステーション
  /camera/image    ──────►         /camera/image
  /lidar/points    ──────►         /lidar/points
  /cmd_vel         ◄──────         /cmd_vel
  /joint_states    (ブロック)
  /diagnostics     (ブロック)
  /tf              (ブロック)
```

#### 2. 同一マシン上の異なるドメイン間通信

ROS 2 は `ROS_DOMAIN_ID` を使って DDS トラフィックを分離します。異なるドメインの
ノードは互いを検出できません。これは分離に有用ですが、ドメイン間で *一部の*
トピックだけを共有したい場合があります — 例えば、ドメイン 1 のシミュレーションと
ドメイン 2 のプランナー間など。

`generic_server` をドメイン 1 で、`generic_client` をドメイン 2 で起動するだけで、
WebSocket は TCP (localhost) 経由で通信するため、DDS ドメイン境界を完全に
バイパスします：

```bash
# ターミナル 1 — ドメイン 1 (例: シミュレーション)
export ROS_DOMAIN_ID=1
ros2 launch ros2_websocket_proxy websocket_server.launch.py

# ターミナル 2 — ドメイン 2 (例: プランナー)
export ROS_DOMAIN_ID=2
ros2 launch ros2_websocket_proxy websocket_client.launch.py \
    ws_url:=ws://localhost:9090
```

YAML に記載されたトピックのみがブリッジされ、それ以外のトピックはそれぞれの
ドメイン内に分離されたままです。

### ワイヤプロトコル

すべての WebSocket バイナリフレームは以下のレイアウトです:

```
オフセット  0                          128              128 + N
           ├──── トピック名 ────────────┤──── CDR ペイロード ──┤
           │ (null パディング, 128 B)    │   (N バイト)         │
```

- **バイト 0〜127**: ROS トピック名（128 バイトまで null パディング）
- **バイト 128〜末尾**: CDR シリアライズされた ROS メッセージ（そのまま転送）

### rosbridge_suite との比較

| 項目 | ros2_websocket_proxy | rosbridge_suite |
|------|---------------------|-----------------|
| シリアライゼーション | CDR (ネイティブバイナリ) | JSON |
| オーバーヘッド | フレームあたり 128 B ヘッダ | JSON エンベロープ + Base64 (バイナリデータ) |
| レイテンシ | 最小限 (memcpy 1 回) | 高い (serialize → JSON → parse) |
| 帯域幅 | ネイティブに近い | バイナリペイロードで 2〜4 倍 (Base64 + JSON) |
| トピック可視性 | 設定したトピックのみ | advertise 後に全トピックが可視 |
| 設定方法 | YAML トピックリスト | 実行時の advertise / subscribe |
| 用途 | マシン間通信、低レイテンシ、トピック分離 | Web ブラウザ統合 |

### 必要条件

- ROS 2 (Humble / Iron / Jazzy 以降推奨)
- C++17 コンパイラ (GCC 9+ または Clang 10+)
- [websocketpp](https://github.com/zaphoyd/websocketpp)
- [Asio](https://think-async.com/Asio/) (スタンドアロン、ヘッダオンリー)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

### インストール

#### 依存パッケージ (Ubuntu)

```bash
cd ~/ros2_ws/src/ros2-websocket-proxy
bash scripts/install_dependency.sh
```

#### ビルド

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_websocket_proxy
source install/setup.bash
```

### 設定

`config/` ディレクトリ内の YAML ファイルを作成・編集します。

**サーバー側** (`config/server_topics.yaml`):

```yaml
subscribe_topics:          # ROS → WebSocket (クライアントへ転送)
  - name: "/camera/image"
    type: "sensor_msgs/msg/Image"

publish_topics:            # WebSocket → ROS (クライアントから受信)
  - name: "/cmd_vel"
    type: "geometry_msgs/msg/Twist"
```

**クライアント側** (`config/client_topics.yaml`):

```yaml
subscribe_topics:          # ROS → WebSocket (サーバーへ転送)
  - name: "/cmd_vel"
    type: "geometry_msgs/msg/Twist"

publish_topics:            # WebSocket → ROS (サーバーから受信)
  - name: "/camera/image"
    type: "sensor_msgs/msg/Image"
```

> **注意** — 一方の `subscribe_topics` はもう一方の `publish_topics` に対応させて
> ください。

### 使い方

#### Launch ファイルを使用

```bash
# サーバー (マシン A)
ros2 launch ros2_websocket_proxy websocket_server.launch.py \
    yaml_file:=server_topics.yaml port:=9090

# クライアント (マシン B)
ros2 launch ros2_websocket_proxy websocket_client.launch.py \
    yaml_file:=client_topics.yaml ws_url:=ws://<サーバーIP>:9090
```

#### ros2 run を使用

```bash
# サーバー
ros2 run ros2_websocket_proxy generic_server \
    --ros-args -p yaml_file:=server_topics.yaml -p port:=9090

# クライアント
ros2 run ros2_websocket_proxy generic_client \
    --ros-args -p yaml_file:=client_topics.yaml -p ws_url:=ws://<サーバーIP>:9090
```

### パラメータ

#### generic_server

| パラメータ | 型 | デフォルト値 | 説明 |
|-----------|------|------------|------|
| `yaml_file` | string | `server_topics.yaml` | YAML 設定ファイル名 (`config/` 以下で解決)。 |
| `port` | int | `9090` | WebSocket サーバーの TCP ポート。 |

#### generic_client

| パラメータ | 型 | デフォルト値 | 説明 |
|-----------|------|------------|------|
| `yaml_file` | string | `client_topics.yaml` | YAML 設定ファイル名 (`config/` 以下で解決)。 |
| `ws_url` | string | `ws://localhost:9090` | 接続先 WebSocket サーバーの URL。 |

### ライセンス

MIT License
