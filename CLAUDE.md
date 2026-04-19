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
| `dsme-beacon-slot-selection-PC.cc` | **Power Control（GPS-based, scheme B）**：純隨機拓樸，同 baseline 結構，加上 GPS table + 因果閘門（模擬 IE 夾帶 GPS） | 完成，待 sweep 驗證 |
| ~~`dsme-beacon-slot-selection-random-pick-backbone-powerControl.cc`~~ | 舊的 RSSI-based 版本（backbone），**計畫刪除** | 淘汰 |

> **GPS-based 實作方式**：目前採「方案 B」— 不改 NS-3 核心，在模擬層維護 `g_gpsTable`，但強制「只有當 rx 收過 tx 的 EB 才能查 tx 的 GPS」（`g_heardFromTxByRx` 因果閘門），避免模擬器上帝視角。這與「真正改 IE 序列化（方案 A）」在指標層面等價。方案 A 未來作為 realism validation 章節補做。

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
N (節點數)    : 5 ~ 50（sweep 時建議每 N 多 seed 取平均）
M (beacon slots): 8（slot 0 保留給 PAN-C，可選 S = {1..7}）
Path loss n   : 2.7（baseline 與 PC 版已統一）
Reference     : d_ref = 1 m, L_ref = 40.05 dB（2.4 GHz FSPL @ 1 m）
RX sens       : -95 dBm
Max Tx        : 0 dBm（PC 的 clamp 上限；PAN-C 固定 0 dBm）
Min Tx        : -32 dBm（PC 的 clamp 下限，PHY PIB 6-bit 兩補數）
PC margin     : 3 dB（預設，可 `--pcMarginDb=X` 調整）
Topology      : 純隨機，PAN-C 置中（±150 m 方形）
```

## 目前進度與待辦

### 已完成
- [x] Baseline 模擬（random topo, 0 dBm fixed）
- [x] **GPS-based power control 模擬（方案 B）** — `dsme-beacon-slot-selection-PC.cc`
- [x] Path loss exponent 統一為 2.7（baseline 與 PC）
- [x] 四項指標實作（p_coll, s̄_coll, η, P̄_tx），PC 版 p_coll/s̄_coll 使用各節點實際 TX
- [x] PAN-C 固定 0 dBm 的設計（bootstrap 種子不做 PC）
- [x] PC 版 N=16 sanity check：p_coll 由 baseline 的 ~0.08 降到 0.018，P̄_tx = −2.43 dBm

### 進行中 / 待完成
- [ ] **跨 N sweep**（N=5..50，每 N 多 seed），畫 baseline vs PC 的四指標對照圖
- [ ] 小 N trade-off 現象的解釋與論文討論（見下方「已知現象」）
- [ ] **Fixed low power 對照組**：複製 PC 版改成「所有 joiner 固定 −10 dBm、−15 dBm」兩組，回答 null hypothesis「是不是只要功率變低就會改善？」。沒有這個對照，reviewer 會說 PC 演算法沒有貢獻、只是剛好功率變低。
- [ ] 刪除舊的 `dsme-beacon-slot-selection-random-pick-backbone-powerControl.cc`
- [ ] **方案 A**（改 NS-3 核心 Custom IE）作為 realism validation — 次要優先
- [ ] 驗證模擬結果 vs 解析式是否吻合

## 已知現象與設計取捨

### PC 在小 N 可能略差於 baseline（trade-off，非 bug）
N ≤ slot 數時，baseline 因為 0 dBm 覆蓋 150 m 全域 → 所有 joiner 聽得到彼此 → 完美協調，p_coll ≈ 0。
PC 縮小發射範圍的同時也**縮掉協調資訊**：後起 joiner 可能聽不到前輩的低功率 beacon → 誤選已占用的 slot → 在某個共同 receiver 處被計為碰撞。
**論文應誠實呈現 crossover**：小 N 持平或略差，大 N（N > slot 數，baseline 進入崩潰區）PC 大勝。

### 方案 B 的因果閘門設計
`g_gpsTable` 是物理真值（mobility install 時登記），但**只能透過 `ComputePcTxDbm` 經 `g_heardFromTxByRx` 閘門存取**。這確保：節點 r 要先收到 t 的 EB，才「知道」t 的 GPS — 在論文層面等價於 GPS 嵌在 beacon IE 中傳遞。

### Short address 失效的坑
LR-WPAN joiner 完成 association 後 MAC 層 short address 會被 coordinator 重新指派，初期建立的 `g_shortToNodeId` 快取會過時。PC 版靠 `LookupNodeIdByShort()` 在 cache miss 時 rescan live device 重建。Baseline 雖然也有同樣問題但靠 DBAN 通知路徑（不需 short addr 查表）繞過去，所以 baseline 看起來沒壞。**若之後新增任何依賴 beacon sender 身分的邏輯，記得用 `LookupNodeIdByShort()` 而非直接 `g_shortToNodeId.find()`**。

## 關鍵程式架構

### Baseline 碰撞偵測邏輯
- `dsme-beacon-slot-selection-baseline.cc:214~246` — p_coll / s̄_coll（per receiver-slot 可見 tx 計數）

### PC 核心邏輯（`dsme-beacon-slot-selection-PC.cc`）
- `ComputePcTxDbm()` — 從 `g_heardFromTxByRx[me]` 找最近鄰居，套 log-distance 反推最小 TX power
- `OnMacRxWithContext()` BEACON 分支 — 經 `LookupNodeIdByShort` 解析 sender，更新因果閘門
- Joiner lambda `*done = true` 之後 — 先 `ComputePcTxDbm → ApplyNodeTxDbm`，再 `CoordBoostrap`，確保第一顆 beacon 就用 PC power
- `PrintSummary` — p_coll/s̄_coll 使用 `g_txDbmByNode[coordId]` 計算可見性；P̄_tx 對 T 集合做 mW 平均再轉 dBm

### PC 版新增 CLI 旋鈕
```
--pcMarginDb=3.0       # fade margin (dB)
--txMinDbm=-32.0       # PC 下限
--txMaxDbm=0.0         # PC 上限
--panCoordTxDbm=0.0    # PAN-C 固定功率（不做 PC）
```

## NS-3 核心修改說明

LR-WPAN DSME MAC 位於 `src/lr-wpan/`。目前 EB 固定序列化 `DsmePANDescriptorIE`，若要加入 GPS Custom IE 需修改：
- `src/lr-wpan/model/lr-wpan-mac-pl-headers.cc` — IE 序列化/反序列化
- `src/lr-wpan/model/lr-wpan-mac.cc` — EB 組建與解析流程

## 論文章節對應實作狀態

| 章節 | 內容 | 實作狀態 |
|------|------|----------|
| Ch3 | Baseline 碰撞機率解析式 | 待推導驗證 |
| Ch4 | GPS-based power control 設計 | 模擬層方案 B 完成；方案 A（真正改 IE 序列化）列為未來章節 |
| Ch4 | Power control 碰撞機率解析式 | 待推導 |
| Ch5 | NS3 模擬比較（Baseline vs PC） | 單點 sanity 完成；跨 N sweep + 多 seed 平均待做 |

## 事件紀錄

### 2026-04-12：本地 git 倉庫毀損（已修復，零資料損失）
- **症狀**：`.git/objects/` 內 5 個 loose object（ac180a85、15c07fd3、1f1e8ade、b7241921、b9439a58）全部為 0 bytes，時間戳一致為 4/12 18:55。`git status`、`git log` 無法執行，所有 git 操作卡住。
- **原因**：4/12 18:55 執行 `git commit` 寫入過程中系統異常中斷（斷電/強制關機），filesystem 只同步了 metadata（檔名）、來不及同步 data（內容），導致 5 個 object file 空殼化。這次 commit 包含方案 B 完成後（4/11 `851a5c75` push）繼續開發的 fixed-low-power 對照組。
- **修復流程（2026-04-19）**：
  1. `cp -r .git .git.broken-backup-20260419`（保險備份）
  2. 把 `refs/heads/dsme_new` 從壞掉的 `ac180a85` 改指向遠端最新 `851a5c75`
  3. 刪除 5 個空 loose object
  4. `git read-tree HEAD` 重整 index
- **資料保全**：working directory 完全沒受影響（那些檔案是編輯器更早就存檔的）。遠端 GitHub 也完全沒受影響。唯一「真的丟失」的是那次 commit 的 metadata（commit message、tree 結構），但**修改的檔案內容仍完整**。
- **教訓**：
  - Commit 完盡快 push，遠端是最可靠的備份
  - 進階防護：`git config --global core.fsync committed-objects,loose-object`（犧牲少量效能換安全）
  - `.git.broken-backup-20260419/` 已加入 `.gitignore`，不 commit 上 repo

### 2026-04-19：方案 A 研究啟動準備
- 當天確認方案 B（scheme B，模擬層 GPS 表 + 因果閘門）已完整 push 至 GitHub（`851a5c75`）。
- 4/11 → 4/12 期間完成但未 push 的工作：
  - `scratch/dsme-beacon-slot-selection-fixed-low-power.cc`（固定低功率對照組，CLAUDE.md TODO 之一）
  - 對應 sweep 腳本更新與 `-5 / -10 / -15 dBm` 三組輸出檔
  - CLAUDE.md 本身的大幅擴寫（指標定義、trade-off 說明、架構段落）
- 這些成果於 2026-04-19 修復 git 後補 commit + push。
- 準備進入方案 A（改 NS-3 核心 `src/lr-wpan/` 的 IE 序列化，把 GPS 真的塞進 Enhanced Beacon 的 Custom IE），將開新 branch `dsme_scheme_a` 進行。
