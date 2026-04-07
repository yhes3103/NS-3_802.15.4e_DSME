# CLAUDE.md — NS-3 DSME Beacon Power Control Research

## 論文資訊
- **題目**：Adaptive Beacon Power Control for Collision Reduction in IEEE 802.15.4e DSME
- **作者**：王承翰
- **核心主張**：透過在 beacon IE 夾帶 GPS 座標，讓每個節點計算最近鄰居距離，以 path loss model (n=2.7) 決定最小 Tx power，縮小干擾半徑，降低 beacon collision 機率

## 專案環境
- **NS-3 版本**：本地修改版，位於 `/home/ns3/NS-3_802.15.4e_DSME/`
- **主要開發 branch**：`dsme_new`
- **Build 指令**：`./ns3 build`
- **執行指令**：`./ns3 run scratch/<filename>`

## 主要模擬檔案（scratch/）

| 檔案 | 說明 | 狀態 |
|------|------|------|
| `dsme-beacon-slot-selection-baseline.cc` | **Baseline**：純隨機拓樸，無 backbone，固定 0 dBm，用於對照組 | 完成 |
| `dsme-beacon-slot-selection-random-pick-backbone-powerControl.cc` | **Power Control（RSSI-based）**：有 backbone，以宣告 TX power + path loss 估距 | 完成但待替換 |

> **重要**：論文提出的是 **GPS-based** power control，目前實作是 RSSI-based 的 proxy 版本（不改 NS-3 核心）。最終版需改為 GPS 座標夾帶於 beacon IE。

## 研究指標定義

### 符號定義

| 符號 | 說明 |
|------|------|
| R | 觀測接收者集合（成功升級的 coordinator） |
| S | Beacon slot 集合，S = {1,…,M-1}（排除 slot 0） |
| T | 會發送 beacon 的節點集合（transmitters） |
| Tₛ ⊆ T | 選擇在 slot s 發送的節點集合 |
| kᵣₛ | receiver r 在 slot s 同時收到的 beacon 數 |
| Pₜ | 節點 t 的發送功率（dBm） |
| P_min | 接收靈敏度門檻（RX sensitivity） |
| L(d) | Log-distance path loss model（dB） |
| d_{t,r} | 節點 t 與接收者 r 之間的距離 |
| 1_vis(t,r) | 可見指示函數（接收功率 ≥ P_min 則為 1） |

### Metric A：p_coll（碰撞機率）

- **碰撞事件**：C_{r,s} = 1 若 kᵣₛ ≥ 2，否則 0
- **公式**：p_coll = (1 / |R||S|) × ΣΣ C_{r,s}
- **值域**：[0, 1]
- 節點數增加 → kᵣₛ 變大 → p_coll 上升；Power control 縮小可見範圍 → p_coll 下降
- 分母 |R| 只含成功升級的節點，失敗節點不計入

### Metric B：s̄_coll（碰撞嚴重度）

- **定義**：S_{r,s} = max(0, kᵣₛ − 1)
- **公式**：s̄_coll = (1 / |R||S|) × ΣΣ S_{r,s}
- **值域**：[0, ∞)，代表平均多餘干擾者數
- 比 p_coll 更敏感：當 p_coll 飽和為 1 時，s̄_coll 仍能反映改善幅度
- 範例：k=2 → S=1（輕微）；k=4 → S=3（嚴重）

### Metric C：η（升級成功率）

- **公式**：η = |R| / N
- 失敗原因：slot 耗盡（附近 slot 皆被佔用）或孤立節點（聽不到任何 beacon）
- Power control 縮小干擾 → 碰撞少 → slot 耗盡降低 → η 上升

### Metric D：P̄_tx（平均發送功率）

- **公式（dBm 直接平均）**：P̄_tx = (1/|T|) Σ Pₜ
- **建議換算 mW 再平均**：P̄_tx,mW = (1/|T|) Σ 10^(Pₜ/10)
- Baseline 固定 0 dBm（1 mW）；Power control 預期 P̄_tx < 0 dBm

### 總覽

| 指標 | 符號 | 值域/單位 | 用途 |
|------|------|-----------|------|
| 碰撞機率 | p_coll | [0,1] | 主要比較指標，跨 N、M 均可用 |
| 碰撞嚴重度 | s̄_coll | [0,∞) | 補充 p_coll 飽和時的差異 |
| 升級成功率 | η | [0,1] | 展示 power control 的額外好處 |
| 平均發送功率 | P̄_tx | dBm/mW | 量化能量節省效益 |

## 模擬參數

```
BO=6, SO=3, MO=5
N (節點數)    : 15 ~ 50
M (beacon slots): 8（16 待定）
Path loss n   : 3.0（程式中），論文擬定 2.7（待統一）
Max Tx        : 0 dBm
Topology      : 純隨機，PAN-C 置中
```

## 目前進度與待辦

### 已完成
- [x] Baseline 模擬（random topo, 0 dBm fixed）
- [x] RSSI-based power control 模擬（含 backbone）
- [x] 四項指標實作（p_coll, s̄_coll, η, P̄_tx）
- [x] Sweep 腳本（`sweep-random-topo.sh`, `collision-sweep-pc.sh`）

### 進行中 / 待完成
- [ ] **GPS-based power control 實作**：將 RSSI-based 替換為真正的 GPS 座標 IE 方式
  - 方案A：修改 NS-3 核心 LR-WPAN MAC（加入 Custom Header IE）
  - 方案B：模擬層 workaround（維護 global GPS table，接收 EB 時查表取座標）
- [ ] 移除 GPS 版本的 backbone 結構（純隨機拓樸）
- [ ] 統一 path loss exponent 為 2.7
- [ ] 推導 Baseline / Power Control 碰撞機率解析式（Chapter 3 & 4）
- [ ] 驗證模擬結果 vs 解析式是否吻合
- [ ] 完整 N=15~50、M=8 的 sweep 比較圖

## 關鍵程式架構

### 碰撞偵測邏輯位置
- `dsme-beacon-slot-selection-baseline.cc:259~316` — Method-1（曾碰撞接收者數）、Method-2a/2b（receiver-slot 碰撞次數/嚴重度）

### Power Control 核心邏輯
- `dsme-beacon-slot-selection-random-pick-backbone-powerControl.cc:287` — 接收 EB 時讀取對方宣告 TX，估算距離
- `dsme-beacon-slot-selection-random-pick-backbone-powerControl.cc:182` — 計算歐氏距離（現為座標直接計算，非 IE 夾帶）

## NS-3 核心修改說明

LR-WPAN DSME MAC 位於 `src/lr-wpan/`。目前 EB 固定序列化 `DsmePANDescriptorIE`，若要加入 GPS Custom IE 需修改：
- `src/lr-wpan/model/lr-wpan-mac-pl-headers.cc` — IE 序列化/反序列化
- `src/lr-wpan/model/lr-wpan-mac.cc` — EB 組建與解析流程

## 論文章節對應實作狀態

| 章節 | 內容 | 實作狀態 |
|------|------|----------|
| Ch3 | Baseline 碰撞機率解析式 | 待推導驗證 |
| Ch4 | GPS-based power control 設計 | 待實作（目前為 RSSI proxy） |
| Ch4 | Power control 碰撞機率解析式 | 待推導 |
| Ch5 | NS3 模擬比較（Baseline vs PC） | 部分完成，待 GPS 版本完成後重跑 |
