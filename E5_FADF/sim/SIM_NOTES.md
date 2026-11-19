# E5_FADF — CCES 2.9.3 模拟器（无目标板）跑通记录

本目录用 `cc21k + ChipFactory 指令集模拟器` 在没有 SC589 目标板的情况下完成
“编译 → 加载 → 运行 → 读回内存”的最小闭环验证。

日期：2026-11-19（会话内所有命令均为实测）

---

## 0. 结论（TL;DR）

| 里程碑 | 状态 | 证据 |
|---|---|---|
| cc21k 编译 sim_hello.c | ✅ 通过 | `sim_hello.doj` 生成 |
| cc21k 链接 sim_hello.dxe | ✅ 通过 | `sim_hello.dxe` 61840 B，elfsyms 有 gOut/gCycles/gPattern |
| 模拟器**加载并执行**该 dxe | ✅ 通过（Functional-Sim） | `chipfactory -exit-value` 自检码 0/1 可复现地随 main 返回值变化，见 §3 |
| Cycle-Accurate-Sim 启动 | ❌ 失败 | `0xC0000409`，见 §5.1 |
| CCES_runner `-d` 导出内存 | ❌ 失败 | 启动即 `Failed to enable tracing log ... %LOCALAPPDATA%\Analog Devices`，见 §4 |
| cycle_count 测 cycles | ⚠️ Functional-Sim 下 `gCycles[0] == 0` | Functional-Sim 不建模 SHARC EMUCLK 周期计数器；需 CA-Sim |

**一句话根因**：两个可执行体（`CCES_runner.exe`、`chipfactory.exe` 非 quiet 模式）
在启动/仿真前都会**无条件创建 `%LOCALAPPDATA%\Analog Devices`**（trace log / 许可证
trusted storage）；本会话 DSH 沙箱只允许写
`D:\DSP_experiment\E5_FADF`（+ 重定向的 TEMP），该目录创建被拒绝 → C++ 未捕获异常 →
进程以 `0xC0000409` 终止。`C:\ProgramData\FLEXnet` 也不存在（CA-Sim 的 FlexNet
trusted storage 缺失），CA-Sim 同样中止。
**这不是代码/工程/命令字符串的问题**（所有显式错误信息都指向环境权限，§5）。

---

## 1. 精确的 target / platform / processor / core 字符串

`CCES_runner.exe --list` 中与本任务相关的组合：

```
Target:   ChipFactory Simulator
Platform: ADSP-SC589 Cycle-Accurate-Sim     <- 指令集 + 周期精确（测 cycles 用这个）
Platform: ADSP-SC589 Functional-Sim         <- 仅功能/指令级（不建模周期计数器）
Processor: ADSP-SC589
```

* **core id = 1**。`System\Chipfactory\xml\Cycle-Accurate-Sim\ADSP-SC589\chipfactory.xml`
  里只有一个实例：`<instance module="SHARC_XI" name="SHARC_XI" coreid="1" coretype="SHARC">`，
  即 CA-Sim 的 SC589 平台**只有 SHARC Core1，没有 ARM core**，所以加载用 core 1，
  `-c` 写成 `-c "1,<dxe>"`（Functional-Sim 的 xml 同样以 Core1 SHARC 为主）。
* `--list --target ... --platform ...` 不会真的过滤，也不显示 processor 列表（实测），
  只能拿到 target/platform 对。
* **processor 字符串就是 `ADSP-SC589`**（不是 `ADSP-SC589 SHARC Core1`；
  用错时 chipfactory 会报 `ERROR: Unknown target architecture '<x>', couldn't open
  '<...>/xml/<sim-type>/<x>/chipfactory.xml'`）。

---

## 2. 编译 + 链接（✅ 已验证）

reference 工程（只读引用，绝不修改）：
`C:\Analog Devices\ADSP-SC5xx_EZ-KIT_Lite-Rel2.0.2\ADSP-SC5xx_EZ-KIT\Examples\drivers\adc\adau1979\adc_dac_playback\SC589\sharc\adc_dac_playback_Core1`
下文记作 `<REF>`。

### 2.1 编译（选项与参考工程 Debug/src/subdir.mk 完全一致）

```
cc21k -c -proc ADSP-SC589 -si-revision any -g
      -DCORE1 -D_DEBUG -DADI_DEBUG
      -structs-do-not-overlap -no-const-strings -no-multiline -warn-protos
      -double-size-32 -char-size-8 -swc
      -I"<REF>\system"
      -I"C:\Analog Devices\ADSP-SC5xx_EZ-KIT_Lite-Rel2.0.2\ADSP-SC5xx_EZ-KIT\include"
      -I"C:\Analog Devices\CrossCore Embedded Studio 2.9.3\SHARC\include"
      -o sim_hello.doj sim_hello.c
```

（`-g` 让 `-d` 能用符号名定位 gOut 等全局；Debug 语义 = 默认 -O0，
因此下面 10000 次循环的 cycles 属于“慢但可比”的 Debug 口径。）

### 2.2 链接（复用参考工程已编好的 startup object + app.ldf，只读）

注意 .doj 在 `Debug\system\...`，app.ldf 在**源码树** `system\startup_ldf\app.ldf`：

```
cc21k -proc ADSP-SC589 -si-revision any
      -T  "<REF>\system\startup_ldf\app.ldf" -no-mem
      -L  "<REF>\Debug\system\startup_ldf"
      -flags-link -MDCORE1,-MDDEBUG,-MDOTHERCORE
      -flags-link -i,<REF>\system
      -add-debug-libpaths
      -o sim_hello.dxe sim_hello.doj
      "<REF>\Debug\system\startup_ldf\app_IVT.doj"
      "<REF>\Debug\system\startup_ldf\app_heaptab.doj"
      "<REF>\Debug\system\startup_ldf\app_startup.doj"
      -flags-link -od,.
```

要点：
* `app_startup.doj` 会调用 `adi_initialize()`，**必须在 sim_hello.c 里提供空实现**
  （`int adi_initialize(void){return 0;}`），否则链接缺符号。
* 不需要 `adi_initialize.doj` 和 EZ-Kit 驱动 .doj（那是外设初始化，模拟器无外设）。
* app.ldf 直接用参考工程的（含 SC589 L1/L2/DDR 内存映射）。没试自写 LDF：复用这个
  LDF 一次成功。

### 2.3 验证结果

```
sim_hello.doj   8784 B
sim_hello.dxe  61840 B
elfsyms sim_hello.dxe:
  gOut. = 0x241330;  gCycles. = 0x241430;  gPattern. = 0x241440;
  gVersion. = 0x241450;  gResult. = 0x241454;  sim_hello_done. = 0x1C07BC;
  __exit_value = 0x90525;
```
`gOut` 与 `gCycles` 相邻（0x241330 + 64*4 = 0x241430），所以一次 `-d ... gOut,80,1,...`
就能同时导出 64 个 float + 4 个 uint32。

脚本：`build_hello.ps1`（`-Cces/-Ref/-SdkInc/-Sim` 可覆盖），实测 `DXE ok`。

---

## 3. 模拟器运行（✅ 加载+执行已验证，Functional-Sim）

### 3.1 独立模拟器 CLI（不走 runner）

`<CCES>\System\chipfactory.exe` 是真正的模拟内核，可直接命令行用：

```
"C:\Analog Devices\CrossCore Embedded Studio 2.9.3\System\chipfactory.exe" ^
  -xml-path "C:\Analog Devices\CrossCore Embedded Studio 2.9.3" ^
  -sim-type "Functional-Sim" -proc "ADSP-SC589" ^
  -quiet -exit-value -bound-exe-time 5000000 sim_hello.dxe
```

* `-xml-path` 要指向 **CCES 安装根**（工具自己拼
  `<xml-path>/System/Chipfactory/xml/<sim-type>/<proc>/chipfactory.xml`）。
  指向 `System\Chipfactory` 会变成 `.../Chipfactory/System/Chipfactory/...` 而失败。
* `-quiet` 在 AppData 不可写时**必须**加：不加就会去建 trace log 目录并直接
  0xC0000409 崩溃；加了之后 Functional-Sim 能完整跑完。
* `-bound-exe-time <cycles>` 防止 `main return` 之后 runtime 的 `__lib_prog_term`
  死循环（默认 0 = 无限）。
* `-exit-value` 让 chipfactory 把程序里的 `__exit_value`（C 运行时 `exit()` 写入）
  作为自己的退出码返回；**0 = 应用退出码 0，1 = 应用退出码非 0**（实测映射表见下）。
* chipfactory 还内置交互式调试命令（`help / run / hal / step / print / peek / poke /
  breakpoint / quit`，从 stdin 读）；但配合 `-quiet` 时实测无任何回显，
  且该 shell 在普通 `-quiet` 运行下不会自动进入 → 不能用作数据导出通道。

### 3.2 执行证据（三个 dxe 对照，同一台机器同一命令）

| 被测 dxe | main 返回值 | Functional-Sim `-exit-value` 退出码 | 说明 |
|---|---|---|---|
| `_tmp\ret0.dxe` | `return 0;` | **0** | 应用退出码 0 |
| `sim_hello.dxe` | `0x2A + gResult + (gCycles[0]!=0 ? 0x100 : 0)` = 42 或 298 | **1** | 应用退出码非 0 |
| `_tmp\cyc.dxe` | `(gCycles[0] != 0) ? 0 : 1` | **1** | ⇒ **`gCycles[0] == 0`** |

结论：
1. dxe 被真正加载并执行到 `main` 结束（返回码变化严格跟源码返回表达式对应），
   **不需要目标板、不需要 Core0 preload**（CA-Sim xml 里本来就没有 ARM core）。
2. **Functional-Sim 不建模周期计数器**（SHARC EMUCLK 读 0），所以
   `cycle_count.h` 在这个平台下测得 0。要测真实 cycles 必须用 Cycle-Accurate-Sim。
3. `-exit-value` 只有 1 bit 可用信息（0/non-0），**不能**用来搬数组，
   只能当“程序真的跑完了”的存活证明。

---

## 4. 内存导出（`-d`）：唯一可行路径是 CCES_runner，但本沙箱被卡死

### 4.1 命令（在 AppData 可写的环境下应可直接用）

```
"C:\Analog Devices\CrossCore Embedded Studio 2.9.3\CCES_runner.exe" ^
  --target "ChipFactory Simulator" ^
  --platform "ADSP-SC589 Functional-Sim" ^
  --processor "ADSP-SC589" ^
  -c "1,D:\DSP_experiment\E5_FADF\sim\sim_hello.dxe" ^
  -d "1,out,D:\DSP_experiment\E5_FADF\sim\hello_out.txt,hex,gOut,80,1,sim_hello_done" ^
  --timeout 30000
```

`-d` 的字段（`--help` 原文）：
`<core>,<dir>,<filename>,<format>,<symbol>,<length>,<stride>,<symbol-to-trigger>`
* `<dir>`：`in`/`out`
* `<format>`：`binary`、`binary:be`，或 `--list-memformats <core>` 列出的格式名；
  本机 `--list-memformats 0` 也被同一个 AppData 崩溃挡住，**没能枚举出格式名**，
  上面先用 `hex`（CCES IDE “Export Memory” 的 8-bit hex 习惯写法）。
* `<symbol>`：符号名或十六进制地址（用 `gOut`，长度 80 覆盖 gOut[64]+gCycles[4]）
* `<length>`：`all` 或个数；`<stride>`：1
* `<symbol-to-trigger>`：命中断点时做读写；这里用源码末尾故意放的
  `sim_hello_done()`（不被内联的 `nop` 函数）当触发器，保证导出发生在数据写好之后。

可选：`--no-halt`、`-b "1,sim_hello_done"`、`--timeout`（毫秒）。

### 4.2 实测错误（原文）

```
Error: Failed to enable tracing log. boost::filesystem::create_directories:
拒绝访问: "C:\Users\24026\AppData\Local\Analog Devices".
进程退出码 -1073740791 (0xC0000409)
```

同样的错误在 `--list-memformats 0`、`-c` 加载、`-d` 导出时都会先出现；
CA-Sim 平台必崩，Functional-Sim 平台偶尔“打错误但退出码 0”，但**没有生成任何数据文件**。

### 4.3 导出文件格式（CCES 约定，verify_hello.py 已实现解析）

* 纯文本，**每行一个字节**，小端；
* 每行是 2 位十六进制（`3f` / `0x3f`）或十进制（0–255）；
* 无地址列、无表头（本目录 `hello_out.txt` 里 `#` 开头的行是本会话加的注释，
  解析脚本会跳过；真实导出不会有）。

`hello_out.txt` = `gOut[64]` 的 256 字节 + `gCycles[4]` 的 16 字节（272 行数据）。
**注意：本目录这份是“格式示例”（文件头已注明 SYNTHETIC），不是模拟器真实导出**，
因为 §4.2 的崩溃导致本次会话无法产出真数据。`verify_hello.py` 对它的解析/校验已跑通。

---

## 5. 失败原因分析（含已排除项）

### 5.1 已排除：不是代码/工程问题
* 编译、链接全绿；dxe 符号齐全；`-proc/-si-revision/-T/-L` 与参考工程 Debug 一致。
* 模拟器**能**按 `-proc ADSP-SC589` 找到 SC589 的 chipfactory.xml（写错 proc 或
  xml-path 时会给完全不同的明确报错，见 §1/§3.1）。
* dxe 路径错误时 runner 给 `Cannot find valid DXE file at ...`，
  证明参数解析、文件探测都正常。
* Functional-Sim 下程序确实执行完（§3.2），说明工具链输出是合法可加载镜像。

### 5.2 根因：环境权限（本会话 DSH 沙箱）
* 沙箱工作区 = `D:\DSP_experiment\E5_FADF`，另加一个重定向 TEMP；
  `C:\Users\24026\AppData\Local` 写入被拒（实测 `mkdir`/`Set-Content` 都是“拒绝访问”）。
* `CCES_runner.exe` 启动时无条件
  `create_directories("%LOCALAPPDATA%\Analog Devices")`（TPSDK trace log），
  异常未被吞掉 → 0xC0000409；`TpsdkServer` 的 `EnableTraceLog(enable, path)`
  由调用方传路径，没有环境变量/命令行开关可改写。
* `chipfactory.exe` 非 `-quiet` 同样会崩；CA-Sim 即使 `-quiet` 也崩，
  与 `C:\ProgramData\FLEXnet`（FlexNet trusted storage）不存在一致 ——
  CA-Sim 的许可证/可信存储初始化也要写系统目录。
* 尝试过并无效：设置 `$env:LOCALAPPDATA`/`$env:APPDATA`（工具用 Known Folder API，
  不读环境变量）、`-xml-path` 变体、Functional-Sim 代替 CA-Sim、piped 交互命令、
  `--list-memformats` 枚举、`-bound-exe-time`、`-no-halt`。

### 5.3 环境事实（本机）
* `C:\Users\24026\AppData\Local\Analog Devices`：**不存在且沙箱内无法创建**
* `C:\ProgramData\Analog Devices`：存在
* `C:\ProgramData\FLEXnet`：**不存在**（CA-Sim 可信存储缺失）

---

## 6. 下一步（需要沙箱外权限，或人工操作一次）

1. **最省事**：在普通 PowerShell（非本 agent 沙箱）里执行一次
   ```
   mkdir "%LOCALAPPDATA%\Analog Devices"
   ```
   然后 `powershell -ExecutionPolicy Bypass -File .\run_hello.ps1 -Backend runner`
   应能真正导出 `hello_out.txt`；`python verify_hello.py hello_out.txt` 校验。
2. 要**测 cycles**：必须让 CA-Sim 能启动 → 需要 FlexNet 可用
   （`C:\ProgramData\FLEXnet` 可写 + CCES 许可证），否则退回 CCES GUI：
   Debug Configurations → C/C++ Application → Target 选
   `ChipFactory Simulator / ADSP-SC589 Cycle-Accurate-Sim`，在 Memory 窗口选中
   `gCycles` 后 “Export Memory”，格式选 8-bit hex，即可得到上述格式文件。
3. 若 CA-Sim 始终不可用：Functional-Sim 只能验证**功能正确性**（gOut 图案、
   gCycles[1..3] 元数据），cycles 需要用周期模型估算或用 E4 实测数据折算，
   不能拿 Functional-Sim 的 0 当结论。

---

## 7. 文件清单

| 文件 | 说明 |
|---|---|
| `sim_hello.c` | 最小测试程序：gOut[64]=i*0.5f、gCycles[4]（含 cycle_count 测量）、gPattern[4]、gVersion、gResult；`adi_initialize()` 空桩；`sim_hello_done()` 导出触发器；`main` 返回自检码 |
| `build_hello.ps1` | 编译+链接（复用参考工程 startup .doj 与 app.ldf，只读）；实测通过 |
| `run_hello.ps1` | 两条后端：chipfactory 自检运行 + CCES_runner `-c/-d` 导出（导出会因 §5.2 失败并打印修复提示） |
| `verify_hello.py` | 解析“每行一个字节/小端”CCES datafile → gOut[64]+gCycles[4]，校验图案与 cycles 合理区间（20000..5000000，0 时提示 Functional-Sim 不建模周期） |
| `hello_out.txt` | **合成格式示例**（非真实导出），用于验证 verify_hello.py |
| `_tmp/` | 本会话探针（ret0/cyc 变体 dxe、解包出的 TpsdkServer wheel 等），可删 |

一键复现：
```
powershell -ExecutionPolicy Bypass -File .\build_hello.ps1
powershell -ExecutionPolicy Bypass -File .\run_hello.ps1
python .\verify_hello.py .\hello_out.txt
```

---

## 8. 本次会话的最终自检输出（`verify_hello.py` 对格式示例）

```
D:\DSP_experiment\E5_FADF\sim\hello_out.txt: 272 bytes
gOut[0..3]   = [0.0, 0.5, 1.0, 1.5]
gOut[60..63] = [30.0, 30.5, 31.0, 31.5]
gOut pattern : PASS
gCycles      = [0, 49995000, 0xFFFFFFFF, 10000]
gCycles meta : PASS
cycles       : 0 -- functional simulator does not model the SHARC core cycle counter (EMUCLK); use Cycle-Accurate-Sim
```

**解析坑（已修）**：CCES 的 8-bit hex 导出写两位十六进制，像 `80` 这种
“只含数字的两位行”会被误当十进制（0x80 被读成 80 = 0x50，浮点会变成 0.8125）。
`verify_hello.py` 现在**整文件判定** hex/decimal：任一行带 `0x` 或 a-f 字母就全按 hex，
否则按十进制；也支持 `#`/`//` 注释行（真实 CCES 导出没有注释）。

## 9. 其它

* 目录里的 `e5_sim_test.c` **不是本任务产出**（并行会话创建），本任务未修改它。
* `linker_log.xml`、`_tmp/`（ret0.dxe / cyc.dxe 探针、解包的 TpsdkServer wheel）
  都是验证过程产物，可安全删除。
* `build_hello.ps1` 会丢弃链接期 stdout；失败时抛异常且 `$LASTEXITCODE != 0`，
  可用 `powershell -ExecutionPolicy Bypass -File .\build_hello.ps1` 直接看输出。
