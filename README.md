# 火箭 6-DOF 冷氣 RCS 姿態控制模擬器

學術級、完全在瀏覽器執行的剛體 6-DOF 火箭姿態控制模擬程式。新手教學見 **[docs/getting-started.md](docs/getting-started.md)**（App 內也有「說明」分頁）；程式架構與資料流見 **[docs/architecture.md](docs/architecture.md)**；所有模型方程式、假設與出處詳見 **[docs/equations.md](docs/equations.md)**；測試驗證結果見 **[docs/verification-report.md](docs/verification-report.md)**。

## 功能

- **6-DOF 動力學**：四元數姿態 + Newton–Euler 剛體方程（含慣量積耦合項），RK4 定步長積分。
- **冷氣 RCS**：最多自訂 12 具推力器（安裝位置、推力方向、額定推力、最小衝量、開關延遲、最大占空比、比衝），閥門狀態機模擬真實開關特性，推進劑消耗。
- **推力分配**：Lawson–Hanson 非負最小平方（NNLS）+ 飽和迭代，支援任意推力器幾何。
- **三種控制律**：PID（旋轉向量誤差 + 條件積分抗飽和）、LQR（瀏覽器內以 Newton–Kleinman 疊代解 CARE，與 scipy 交叉驗證）、Quaternion PD（全域大角度）。
- **調變**：PWPF（脈寬脈頻）、PWM（含最小衝量殘差追蹤）、理想連續推力（對照組）。
- **擾動**：初始角速度、常值力矩、種子化隨機力矩、重力梯度（給定圓軌道）、太陽輻射壓。
- **感測器**：陀螺儀雜訊／偏置、姿態量測誤差（控制器使用估計值）。
- **輸出**：Three.js 3D 動畫（含鼻錐火箭模型、箭體軸標籤、質心、目標姿態框架、RCS 排焰、角速度／速度向量、軌跡）、9 組圖表（支援滾輪縮放／拖曳平移／雙擊重設／游標數值提示）：姿態、誤差、角速度、指令 vs 實際力矩、RCS 開關歷程、推進劑與 Δv、平移狀態、相圖（三軸軌跡／總量雙模式）、效能指標（含各推力器統計表）、CSV 匯出、案例 JSON 匯入／匯出、說明頁。
- **三個參考案例**：小型（立方衛星級）／中型上面級／大型高慣量運載火箭。

## 執行

### 線上預覽
本模擬器為純靜態網頁（無後端），可直接部署於任何靜態網頁空間。

### 本機執行

```bash
# 需要本機 HTTP 伺服器（ES modules 不支援 file:// 協定）
cd rocket-rcs-sim
python3 -m http.server 8080
# 瀏覽 http://localhost:8080
```

### 執行回歸測試（不需瀏覽器）

```bash
node test/runTests.mjs
```

15 項測試涵蓋：四元數運動學 vs 解析解、能量／角動量守恆、推力器力學符號、LQR vs scipy 解析解、分配器可控性、三種控制器閉迴路收斂（含全鏈路調變與閥門）、最小衝量強制、步長收斂性。

## 檔案結構

```text
rocket-rcs-sim/
├─ index.html            # 版面骨架（CDN: Three.js / Chart.js）
├─ css/style.css
├─ js/
│  ├─ main.js            # 進入點、模擬迴圈
│  ├─ parameters.js     # 預設參數、三個參考案例
│  ├─ mathUtils.js       # 向量/矩陣/四元數、線性求解、種子化 RNG
│  ├─ dynamics.js        # 6-DOF Newton–Euler（純函數導數）
│  ├─ integrator.js      # RK4
│  ├─ rcs.js             # 推力器模型、閥門狀態機
│  ├─ allocation.js       # NNLS 力矩分配
│  ├─ modulation.js       # PWPF / PWM / 連續
│  ├─ controllers/
│  │  ├─ pid.js
│  │  ├─ lqr.js           # Newton–Kleinman CARE
│  │  └─ quaternionPD.js
│  ├─ disturbance.js      # 重力梯度、SRP、常值/隨機力矩
│  ├─ sensors.js          # 陀螺儀／姿態量測誤差
│  ├─ simulation.js      # 引擎（時鐘架構、記錄、指標）
│  ├─ plot.js             # Chart.js 圖表
│  ├─ scene3d.js          # Three.js 場景
│  └─ ui.js               # 參數編輯器（宣告式面板）
├─ docs/equations.md     # 方程式與出處（學術依據）
└─ test/runTests.mjs      # 15 項回歸測試
```

## 單位與座標系

SI 單位。本體系 \(+X\) = 縱軸（鼻錐），\(Y/Z\) 橫向；原點 = 箭體幾何中心。姿態顯示為 3-2-1 Euler 角（roll–pitch–yaw）。推力器「方向」欄為**作用於箭體的力**方向（排氣方向相反）。

## 主要文獻

- Wertz, J.R. (ed.), *Spacecraft Attitude Determination and Control*, Springer, 1978.
- Wie, B., *Space Vehicle Dynamics and Control*, 2nd ed., AIAA, 2008.
- Markley & Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control*, Springer, 2014.
- Sola, J., arXiv:1711.02508（四元數運動學）。
- 完整出處清單（含 URL）：[docs/equations.md](docs/equations.md)。
