# 模型方程式與出處（Model Equations and Sources）

本文件為本模擬器所有計算過程的嚴謹依據。每一節對應程式模組，方程式編號貫穿程式碼註解。

座標系與符號約定：

- 慣性系 \(I\)（World）、本體系 \(B\)（Body）。箭體縱軸 = 本體系 \(+X\)（鼻錐方向）。
- 姿態四元數 \(q=[q_w,q_x,q_y,q_z]\)（純量在前），代表 body→inertial 主動旋轉：\(v_I = R(q)\,v_B\)。
- \(\boldsymbol{\omega}\) 為本體系角速度；\(J\) 為對質心的本體系慣量張量。
- 質心位置 \(\mathbf{c}\)（本體系，原點為箭體幾何中心）；推力器安裝位置 \(\mathbf{p}_i\)（本體系）。

---

## §1 剛體 6-DOF 動力學（`dynamics.js`）

### E1.1 質心平移（Newton）

\[
\dot{\mathbf r} = \mathbf v,\qquad
\dot{\mathbf v} = \frac{R(q)\,\mathbf F_B + \mathbf F_{\text{ext},I}}{m},\qquad
m = m_{\text{dry}} + m_{\text{prop}}
\]

\(\mathbf F_B\) 為 RCS 合力（本體系），經 \(R(q)\) 轉至慣性系。

### E1.2 慣量張量

\[
J=\begin{bmatrix}
I_{xx} & -I_{xy} & -I_{xz}\\
-I_{xy} & I_{yy} & -I_{yz}\\
-I_{xz} & -I_{yz} & I_{zz}
\end{bmatrix}
\]

使用者輸入慣性積（\(P_{xy}=\int xy\,dm\) 等，正號），張量非對角項為 \(-P_{xy}\)。UI 進行正定性（Cholesky）與主軸三角不等式（\(I_{\max}<I_2+I_3\)）檢查，確保剛體物理可實現。

### E1.3 姿態動力學（Euler 方程）

\[
J\dot{\boldsymbol{\omega}} + \boldsymbol{\omega}\times(J\boldsymbol{\omega}) = \boldsymbol{\tau}_B
\quad\Longrightarrow\quad
\dot{\boldsymbol{\omega}} = J^{-1}\left(\boldsymbol{\tau}_B - \boldsymbol{\omega}\times(J\boldsymbol{\omega})\right)
\]

\(\boldsymbol{\tau}_B\) 含 RCS 力矩與外部擾動力矩。此為標準剛體 Newton–Euler 方程。

**出處：**
- Euler 剛體動力學：[Euler's equations (rigid body dynamics) — Wikipedia](https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics))；Wertz, J.R. (ed.), *Spacecraft Attitude Determination and Control*, Springer, 1978, §7.1。
- 慣量張量與慣性積符號約定：[List of moments of inertia — Wikipedia](https://en.wikipedia.org/wiki/List_of_moments_of_inertia#Inertia_tensors)。

## §2 四元數姿態運動學（`mathUtils.js`）

### E2.1 四元數微分方程

\[
\dot q = \tfrac{1}{2}\,q\otimes\begin{bmatrix}0\\ \boldsymbol{\omega}\end{bmatrix}
\]

積分時每步以 RK4 後正規化（\(\|q\|=1\)）。

### E2.2 誤差四元數

\[
q_e = q^{-1}\otimes q_d
\]

\(q_e\) 的向量部分代表「由目前姿態前往目標所需的旋轉」（本體系）。\(q\) 與 \(-q\) 等價（雙重覆蓋），以 \(\operatorname{sign}(q_{e,w})\) 處理。

### E2.3 旋轉向量（log map）

\[
\boldsymbol{\delta} = 2\,\operatorname{atan2}\!\left(\|q_{e,v}\|,\ |q_{e,w}|\right)\frac{q_{e,v}}{\|q_{e,v}\|}
\]

PID 使用 \(\boldsymbol{\delta}\) 作為三軸誤差（避免 Euler 角奇異與直接相減問題）。

**出處：**
- 四元數運動學與慣例：Sola, J., "Quaternion kinematics for the error-state Kalman filter", arXiv:1711.02508, 2017 — <https://arxiv.org/abs/1711.02508>（§2、§3）。
- Wertz (1978), §12.3；Markley, F.L. & Crassidis, J.L., *Fundamentals of Spacecraft Attitude Determination and Control*, Springer, 2014, §1.3。

## §3 冷氣 RCS 模型（`rcs.js`）

### E3.1 推力與力矩

第 \(i\) 具推力器（推力 \(f_i\)，方向單位向量 \(\mathbf d_i\) 為「作用於箭體的力」方向；排氣方向為 \(-\mathbf d_i\)）：

\[
\mathbf F_B=\sum_i f_i\,\mathbf d_i,\qquad
\boldsymbol{\tau}_B=\sum_i (\mathbf p_i-\mathbf c)\times(f_i\,\mathbf d_i)
\]

### E3.2 閥門狀態機

狀態：`CLOSED → PENDING_OPEN →（延遲 t_on）→ OPEN →（關閉指令且已達最小開啟時間）→ PENDING_CLOSE →（延遲 t_off）→ CLOSED`。

- 最小開啟時間 \(t_{\min}=I_{\min}/F_{\max}\)（最小衝量限制）——強制「實際開啟時間」≥ \(t_{\min}\)，而非僅指令寬度。
- 占空比：滑動視窗（預設 10 s）內開啟時間比例不得超過 `maxDuty`，超過則禁止新點火。
- 關閉途中再次要求開啟則直接回到 `OPEN`（不重觸發開延遲）。
- 推進劑耗盡時強制關閉所有閥門（物理截斷最後一個脈衝）。

### E3.3 推進劑消耗

\[
\dot m_p = -\sum_i \frac{f_i}{I_{sp,i}\,g_0},\qquad g_0=9.80665\ \text{m/s}^2
\]

**出處：**
- 冷氣推進特性（比衝、推力等級、最小衝量）：[Cold gas thruster — Wikipedia](https://en.wikipedia.org/wiki/Cold_gas_thruster)；[Reaction control system — Wikipedia](https://en.wikipedia.org/wiki/Reaction_control_system)。
- \(g_0\) 標準值：ISO 80000-3。
- 常數推力／常數 \(I_{sp}\) 為明確命名之近似（非吹降式模型）。

## §4 推力分配（`allocation.js`）

控制器輸出連續等效力矩指令 \(\boldsymbol{\tau}_{cmd}\)，分配器求解有界最小平方：

\[
\min_{\mathbf f}\ \left\|\begin{bmatrix}T\\ \sqrt{w_F}\,F_m\end{bmatrix}\mathbf f-\begin{bmatrix}\boldsymbol{\tau}_{cmd}\\ \mathbf 0\end{bmatrix}\right\|^2
\quad\text{s.t.}\quad 0\le f_i\le F_{\max,i}
\]

其中 \(T\) 的第 \(i\) 欄為 \((\mathbf p_i-\mathbf c)\times\mathbf d_i\)，\(F_m\) 的第 \(i\) 欄為 \(\mathbf d_i\)。\(w_F\) 懲罰非預期淨力（預設 0.1）。

**求解：Lawson–Hanson 非負最小平方法（NNLS）** + 上界飽和迭代。直接偽逆解的負推力分量經裁剪後會收斂到次優解（實測僅達成 50% 指令力矩），故必須使用 NNLS。

可控性檢查：每軸正／負向可達力矩（給極大指令取實際達成值）；軸向滿刻度力矩 \(\hat\tau_a\) 同法求得，供調變器正規化。

**出處：**
- Lawson, C.L. & Hanson, R.J., *Solving Least Squares Problems*, SIAM, 1995（NNLS 演算法第 23 章）。
- 討論推力器開關限制下的調變：Kienitz, K.H., Bals, J., et al., "Pulse modulation for attitude control with thrusters subject to switching restrictions", *Aerospace Science and Technology* 9(7), 2005 — <https://doi.org/10.1016/J.AST.2005.06.006>。

## §5 調變器（`modulation.js`）

### E5.1 PWPF（脈寬脈頻調變）

每軸一組「一階低通濾波器 + 遲滯繼電器」：

\[
\dot u_f=\frac{K_m u-u_f}{T_m},\qquad
u=\frac{\tau_{cmd,a}}{\hat\tau_a}\ (\text{正規化})
\]

繼電器（對稱三態）：\(u_f\ge U_{on}\Rightarrow+1\)；\(u_f\le-U_{on}\Rightarrow-1\)；\(|u_f|\le U_{off}\Rightarrow 0\)（遲滯帶）。輸出 \(\pm\hat\tau_a\)。

### E5.2 PWM（脈寬調變）

固定週期 \(T_{pwm}\)、占空比 \(d=\min(1,|\tau_{cmd,a}|/\hat\tau_a)\)，週期起點相位。當 \(d\cdot T_{pwm}<t_{\min}\) 時採殘差追蹤（pulse skipping）：累積占空比至 ≥ 最小占空比才點火，避免直接捨去形成死區。

### E5.3 理想連續模式

不經調變，推力可連續比例調節——用於驗證控制器本身（分離控制器設計與致動器非理想性）。

**出處：**
- PWPF 結構（\(K_m, T_m, U_{on}, U_{off}\)）與應用：Shokrolahi, S., et al., "Autonomous Spacecraft Docking Under Uncertainty", *Int. J. Aerospace Eng.*, 2023 — <https://onlinelibrary.wiley.com/doi/10.1155/ijae/6523148>（Fig. 3）；Khosravi, A., "Tuning of PWPF Modulator using PSO", *Automatika* 57(1), 2016 — <https://hrcak.srce.hr/165517>。
- 離散推力脈衝捨入與殘差追蹤：Alcorn, J., Schaub, H., Piggott, S., "Attitude Control Performance Analysis Using Discretized Thruster with Residual Tracking", AAS 16-038, 2016 — <https://hanspeterschaub.info/Papers/Alcorn2016.pdf>。
- 閥門延遲對 PWM 脈寬的影響（本模擬之設計約束：PWM 週期必須遠大於總延遲）：見 ASCE RDO 文獻對致動器延遲建模之討論 — <https://ascelibrary.org/doi/10.1061/JAEEEZ.ASENG-5338>。

## §6 控制器（`js/controllers/`）

三種控制律輸出皆為連續力矩指令 \(\boldsymbol{\tau}_{cmd}\)（本體系），經 §4 分配與 §5 調變後驅動閥門。

### E6.1 三軸 PID（`pid.js`）

\[
\boldsymbol{\tau}_{cmd}=K_p\boldsymbol{\delta}-K_d\boldsymbol{\omega}+K_i\int\boldsymbol{\delta}\,dt
\]

\(\boldsymbol{\delta}\) 為 E2.3 旋轉向量誤差（\(\dot{\boldsymbol\delta}\approx-\boldsymbol\omega\)，故微分項為 \(-K_d\omega\)）。積分項含條件積分抗飽和：(1) 力矩飽和（連續分配達成率 < 0.98）時凍結；(2) 大角度機動（\(|\delta_a|>\) 積分視窗，預設 0.1 rad）期間不積分——避免機動過程積分上捲造成反向慢速振盪。

### E6.2 LQR（`lqr.js`）

繞固定姿態／零角速度線性化（小角度）：

\[
\mathbf x=\begin{bmatrix}\delta\boldsymbol{\theta}\\ \boldsymbol{\omega}\end{bmatrix},\quad
A=\begin{bmatrix}0&I_3\\0&0\end{bmatrix},\quad
B=\begin{bmatrix}0\\J^{-1}\end{bmatrix},\quad
\boldsymbol{\tau}=-K\mathbf x
\]

求解連續時間代數 Riccati 方程（CARE）：

\[
A^TP+PA-PBR^{-1}B^TP+Q=0,\qquad K=R^{-1}B^TP
\]

**求解：Newton–Kleinman 疊代**（Kleinman 1968）——初值取可穩定化增益 \(K_0=[\nu^2J,\ 2\zeta\nu J]\)（\(\nu=0.3\)、\(\zeta=0.9\)，形成三個阻尼二階系統），每輪解 Lyapunov 方程

\[
(A-BK_k)^TP_k+P_k(A-BK_k)=-(Q+K_k^TRK_k)
\]

以 36×36 線性系統（\((I\otimes A_k^T+A_k^T\otimes I)\operatorname{vec}(P)=-\operatorname{vec}(Q_c)\)）求解後更新 \(K_{k+1}=R^{-1}B^TP_k\)，至 \(\|K_{k+1}-K_k\|_\infty<10^{-10}\)。驗證：P 正定（Cholesky）、正規化 CARE 殘差 < 10⁻⁶。對角慣量單軸解析解（測試 T5a 基準）：

\[
K_p=\sqrt{q_\theta/r},\qquad K_d=\sqrt{q_\omega/r+2J\sqrt{q_\theta/r}}
\]

已與 `scipy.linalg.solve_continuous_are` 交叉驗證（數值一致至 10⁻⁶）。LQR 最佳性僅適用於局部線性、未飽和模型；大角度機動與脈衝閥門下行為由模擬檢驗。

### E6.3 Quaternion PD（`quaternionPD.js`）

\[
\boldsymbol{\tau}_{cmd}=2K_p\,\operatorname{sign}(q_{e,w})\,\mathbf q_{e,v}-K_d\boldsymbol{\omega}
\]

全域（大角度）非線性回授，具 Lyapunov 穩定性（取 \(V=\frac12\boldsymbol{\omega}^TJ\boldsymbol{\omega}+2K_p(1-|q_{e,w}|)\)，可證 \(\dot V\le0\)）。

**出處：**
- PID 條件積分／抗飽和實務：Åström, K.J. & Hägglund, T., *Advanced PID Control*, ISA, 2006。
- LQR/CARE/Newton–Kleinman：Kleinman, D.L., "On an iterative technique for Riccati equation computation", *IEEE Trans. Automatic Control* 13(1), 1968；[Algebraic Riccati equation — Wikipedia](https://en.wikipedia.org/wiki/Algebraic_Riccati_equation)；Anderson, B.D.O. & Moore, J.B., *Optimal Control: Linear Quadratic Methods*, Prentice Hall, 1990。
- Quaternion PD 與 Lyapunov 分析：Wie, B., *Space Vehicle Dynamics and Control*, 2nd ed., AIAA, 2008, §8.5。

## §7 外部擾動（`disturbance.js`）

### E7.1 重力梯度力矩

\[
\boldsymbol{\tau}_{gg}=\frac{3\mu}{r^3}\,\hat{\mathbf r}_B\times(J\hat{\mathbf r}_B)
\]

\(\mu=3.986004418\times10^{14}\ \text{m}^3/\text{s}^2\)（地球）。採「給定圓軌道」近似：軌道半徑由高度設定，近地方向以軌道率 \(n=\sqrt{\mu/r^3}\) 在慣性系旋轉（不與載具平移耦合，此近似已明確標註）。

### E7.2 太陽輻射壓

\[
\mathbf F_{srp}=-P_{sr}\,C_r\,A\,\hat{\mathbf S}_B,\qquad
\boldsymbol{\tau}_{srp}=\mathbf r_{cp}\times\mathbf F_{srp}
\]

\(P_{sr}=4.56\times10^{-6}\ \text{N/m}^2\)（1 AU）、常數面積、無蝕近似（皆已明確標註）。

### E7.3 常值與隨機力矩

常值力矩直接相加；隨機力矩為零均值高斯白雜訊（固定取樣間隔，種子化 PRNG，可重現）。

**出處：**
- 重力梯度：[Gravity-gradient stabilization — Wikipedia](https://en.wikipedia.org/wiki/Gravity-gradient_stabilization)；Wertz (1978), §16.1。
- 太陽輻射壓常數：[Solar radiation pressure — Wikipedia](https://en.wikipedia.org/wiki/Solar_radiation_pressure)（1 AU 處 4.56 μN/m²）。

## §8 時鐘架構與積分（`integrator.js`、`simulation.js`）

分離時鐘：物理積分（RK4，預設 \(dt=5\) ms）／控制器取樣（預設 0.1 s）／調變器（每物理步）／閥門狀態機（每物理步）／擾動與感測器取樣（各自間隔）／記錄（預設 0.05 s）。

- RK4 各 stage 中，閥門狀態與已取樣擾動保持固定；導數函數為純函數（無副作用）。
- 狀態相關力（重力梯度、SRP）於各 stage 以該 stage 姿態重新計算。
- 四元數於每步完成後正規化。
- 半步長重跑軌跡一致性（測試 T7g）驗證收斂性。

## §9 輸出量定義（`simulation.js`、`metrics.js`）

| 量 | 定義 |
|---|---|
| 姿態誤差角 | \(\|\boldsymbol{\delta}(t)\|\)（E2.3 log map） |
| 穩定時間 | 最後一次 \(\|\boldsymbol{\delta}\|>\) 帶寬（預設 0.5°）的時刻 |
| 累計 Δv | \(\Delta v=\int_0^t \frac{\|F_{RCS,I}\|}{m}\,dt\)（RCS 淨加速度積分） |
| Δv 向量 | \(\|\mathbf v(t)-\mathbf v(0)\|\) |
| 控制能量 | \(\int\|\boldsymbol{\tau}_{actual}\|^2\,dt\) |
| 效能指標 | 穩定時間、最終/RMS/最大誤差、推進劑、循環次數、點火時間 |

注意：純力偶消耗推進劑但淨力為零，故 \(\Delta v\)（累計）可為零而推進劑減少——兩個量不可互換（文件與 UI 均標註）。

## §10 感測器（`sensors.js`）

\[
\hat{\boldsymbol{\omega}}=\boldsymbol{\omega}+\mathbf b+\mathbf n,\qquad
\hat q=\delta q_{noise}\otimes q
\]

陀螺儀雜訊（rad/s/√Hz，取樣間隔折算）、常值偏置、姿態量測雜訊（小隨機旋轉）。控制器使用估計值（非真值）。

## §11 測試驗證（`test/runTests.mjs`）

15 項回歸測試：

1. **T1** 四元數運動學 vs 解析旋轉；\(q\) 與 \(-q\) 動力學等價。
2. **T2** 無力矩旋轉：轉動能量與慣性系角動量守恆（含耦合慣量張量，RK4，誤差 < 10⁻⁸）。
3. **T3/T4** 單推力器力／力矩符號（解析值比對）；純力偶淨力為零。
4. **T5** LQR 對角解析解（與 scipy 一致）；閉迴路穩定性與 CARE 殘差。
5. **T6** 分配器雙向可控性（含質心偏移）。
6. **T7** 三控制器收斂（連續推力）；PID + PWPF／PWM + 閥門延遲／最小衝量／占空比全鏈路收斂；最小開啟時間 ≥ \(t_{\min}\)；半步長重跑一致。

執行：`node test/runTests.mjs`


## §12 已知近似（明確標註之範圍邊界）

- 單一剛體（無撓性、無推進劑晃動）；質量減少時質心與慣量不變（推進劑集中於質心之近似）。
- 常數推力／常數 \(I_{sp}\) 冷氣近似（非壓力吹降模型）。
- 重力梯度採給定圓軌道（不與平移耦合）；SRP 常數面積、無蝕。
- 6-DOF 完整傳播平移，但不含位置控制迴路。

## §13 理論推導補充

本節補充 §1–§7 各模型的推導細節，供需要深入理解的讀者查閱。

### E13.1 四元數運動學的由來

剛體姿態以旋轉矩陣 \(R\)（body→inertial）表示、角速度 \(\boldsymbol{\omega}\) 以本體系表示時，其時間微分為（見 Sola 2017, §2）：

\[
\dot R = R\,[\boldsymbol{\omega}]_\times
\]

其中 \([\boldsymbol{\omega}]_\times\) 為角速度的反對稱矩陣（右乘 = 本體系角速度）。將 \(R\) 以四元數 \(q=(w,\boldsymbol{v})\) 表示並代入上式，整理可得等效的一階 ODE：

\[
\dot q = \tfrac{1}{2}\,q\otimes\begin{bmatrix}0\\ \boldsymbol{\omega}\end{bmatrix}
= \tfrac{1}{2}\begin{bmatrix} -\boldsymbol{v}\cdot\boldsymbol{\omega} \\\ w\,\boldsymbol{\omega} + \boldsymbol{v}\times\boldsymbol{\omega}\end{bmatrix}
\]

本模擬以 RK4 積分後每步正規化（\(\|q\|=1\)），抑制數值漂移。

**為何不用 Euler 角？** 3-2-1 Euler 角在 pitch = ±90° 存在萬向鎖（gimbal lock）奇異：兩個自由度退化為同一軸，角速度到角變率的轉換矩陣（\(T(\phi,\theta)\)）行列式為零。四元數是 \(SO(3)\) 的雙重覆蓋，全域非奇異。**雙重覆蓋**：\(q\) 與 \(-q\) 對應同一物理旋轉，故誤差處理時取 \(\operatorname{sign}(q_{e,w})\) 將旋轉限制在 ≤180° 的最短路徑（避免「繞遠路」的反向轉動）。

### E13.2 Newton–Euler 方程的推導

慣性系角動量 \(\mathbf{H}_I = m\,\mathbf{r}\times\dot{\mathbf{r}} + \int \boldsymbol{\rho}\times\dot{\boldsymbol{\rho}}\,dm\)，其中剛體內部項在本體系可寫成 \(\mathbf{h}_B = J\boldsymbol{\omega}\)。由 \(\mathbf{H}_I\) 守恆（無外力矩時）與 \(\dot{\mathbf{h}}_B + \boldsymbol{\omega}\times\mathbf{h}_B = \boldsymbol{\tau}_B\)（Sola 2017, §3；Wertz 1978, §16）可得本體系 Euler 方程：

\[
J\dot{\boldsymbol{\omega}} = \boldsymbol{\tau}_B - \boldsymbol{\omega}\times(J\boldsymbol{\omega})
\]

其中 \(\boldsymbol{\omega}\times(J\boldsymbol{\omega})\) 為**陀螺耦合項**：即使無外力矩，角速度與慣量軸不對齊時也會產生進動；自由剛體繞最大或最小慣量主軸的旋轉是穩定的，繞中間慣量主軸則不穩定（中間軸定理／網球拍定理）。慣量張量

\[
J = \begin{bmatrix} I_{xx} & -P_{xy} & -P_{xz} \\ -P_{xy} & I_{yy} & -P_{yz} \\ -P_{xz} & -P_{yz} & I_{zz} \end{bmatrix},\qquad
I_{xx}=\int(y^2+z^2)\,dm,\quad P_{xy}=\int xy\,dm
\]

使用者輸入正號慣性積 \(P\)，張量自動取負（§1）。模擬的 T2/T10b 測試驗證無外力時 \(\mathbf{H}_I\) 與動能守恆，即此耦合項正確。

### E13.3 PWPF 調變器的遲滯分析

PWPF 由一階低通濾波器 + Schmitt trigger（遲滯比較器）組成（Kuwao et al. 2007；Habineh 2009 見 §5 出處）：

1. **符號–量值分離**：指令符號單獨處理（僅在點火關閉時可更新方向，避免方向顫動），量值 \(|u|\)（截於 1.5）進入濾波器。
2. **一階低通濾波（含輸出回授，標準 PWPF 拓撲）**：\(\dot u_f = (K_m(|u| - s) - u_f)/T_m\)，其中 \(s \in \{0, 1\}\) 為目前點火狀態——點火時回授使 \(u_f\) 反向衰減至 \(U_{off}\) 而關閉；關閉時 \(u_f\) 朝 \(K_m|u|\) 回升至 \(U_{on}\) 而再點火，形成自然脈衝序列。
3. **一側遲滯繼電器**（Schmitt trigger）：\(u_f \ge U_{on} \Rightarrow s=1\)；\(u_f \le U_{off} \Rightarrow s=0\)；介於其間維持原狀態（遲滯帶）。

**脈衝寬度與週期**（可解析估計）：開啟期間 \(u_f\) 以時間常數 \(T_m\) 衰減至穩態 \(K_m(|u|-1)\)，故脈衝寬度 \(\approx T_m\ln\frac{1-|u|+U_{on}/K_m}{1-|u|+U_{off}/K_m}\)；關閉期間回升至穩態 \(K_m|u|\)，關閉時間 \(\approx T_m\ln\frac{|u|-U_{off}/K_m}{|u|-U_{on}/K_m}\)。由此可見：\(|u| \to U_{on}/K_m\) 時關閉時間發散（脈衝頻率趨零，形成死區邊界）；遲滯帶越寬，頻率越低。調參方向：增大 \(T_m\) 或遲滯帶 → 開關次數下降（省推進劑、閥門壽命長）但響應變慢；減小則反之。此為「控制精度 ↔ 推進劑/閥門壽命」的基本取捨。

**PWM 殘差追蹤**（`modulation.js` 實作）：固定週期 \(T_{pwm}\)、占空比 \(=|u|\)（截於 1）；當需求 on-time \(|u|T_{pwm}\) 低於最小脈衝寬度 \(t_{min}\) 時，以時間為單位累積需求（\(|u|\,dt\)，與物理步長無關），累積至 \(\ge t_{min}\) 時以倒數計時器交付完整累積需求（pulse skipping），消除小指令死區且平均輸出 \(=|u|\)。

### E13.4 LQR 與 Newton–Kleinman 疊代

線性化（`lqr.js` `linearize()`）：狀態 \(x=[\boldsymbol{\varepsilon};\ \boldsymbol{\omega}]\)，其中 \(\boldsymbol{\varepsilon}=-\boldsymbol{\delta}\) 為**偏離目標**的誤差旋轉向量（\(\boldsymbol{\delta}\) 為前往目標的旋轉，見 E2.3），\(\dot{\boldsymbol{\varepsilon}}=\boldsymbol{\omega}\)，\(J\dot{\boldsymbol{\omega}}=\boldsymbol{\tau}\)：

\[
\dot x = Ax + Bu,\qquad
A=\begin{bmatrix}0&I\\0&0\end{bmatrix},\qquad
B=\begin{bmatrix}0\\J^{-1}\end{bmatrix},\qquad u=-Kx
\]

閉迴路（對角慣量時逐軸）：\(J s^2 + K_d s + K_p = 0\)，\(K_p, K_d > 0\) 穩定。控制律 \(\tau = -Kx = K_p\boldsymbol{\delta} - K_d\boldsymbol{\omega}\)（施加力矩往 \(\boldsymbol{\delta}\) 方向轉、同時阻尼角速度）。

最小化 \(\int (x^TQx + u^TRu)\,dt\) 的最佳回授 \(u=-Kx\) 由 CARE 決定：

\[
A^TP + PA - PBR^{-1}B^TP + Q = 0,\qquad K = R^{-1}B^TP
\]

**Newton–Kleinman 疊代**（Kleinman 1968）：給定穩定的初始回授 \(K_0\)（本模擬以 \(\nu=0.3\,\text{rad/s}\)、\(\zeta=0.9\) 的 PD 極點配置生成），重複解 Lyapunov 方程

\[
(A-BK_i)^TP_i + P_i(A-BK_i) + Q + K_i^TRK_i = 0,\qquad K_{i+1} = R^{-1}B^TP_i
\]

直到 \(\|K_{i+1}-K_i\|\) 收斂（殘差指標顯示於 LQR 狀態框）。**對角系統解析解**（驗證用，與 scipy `solve_continuous_are` 一致，含符號）：

\[
K_p = \sqrt{q_\theta/r},\qquad K_d = \sqrt{q_\omega/r + 2J\sqrt{q_\theta/r}}
\]

與 scipy `solve_continuous_are` 交叉驗證一致（見 verification-report.md）。

### E13.5 重力梯度力矩的假設

圓軌道上，箭體兩端所受重力略有差異，產生恢復力矩（Wertz 1978, §16；詳細推導見 Wie 2008, §8.5）：

\[
\boldsymbol{\tau}_{gg} = \frac{3\mu}{r^3}\,\hat{\mathbf{r}}\times(J\hat{\mathbf{r}})
\]

其中 \(\hat{\mathbf{r}}\) 為軌道向徑方向單位向量（以給定軌道高度與初始相位沿圓軌道運動）。本模擬採**給定軌道**（不受姿態影響回饋），屬近似（§12）；長軸對準地心方向為穩定平衡，且需被動或主動阻尼否則振盪。

### E13.6 太陽輻射壓的假設

\[
\boldsymbol{\tau}_{srp} = \mathbf{r}_{cp}\times\left(P\,C_r A\,\hat{\mathbf{s}}\right)
\]

其中 \(P = 4.56\times10^{-6}\,\text{N/m}^2\)（1 AU 處動壓，Wertz 1978）、\(C_r\) 為反射係數、\(A\) 為有效受照面積、\(\mathbf{r}_{cp}\) 為壓力中心相對質心的偏移。採常數面積與瞬時反射（無自轉調變），見 §12 近似清單。
