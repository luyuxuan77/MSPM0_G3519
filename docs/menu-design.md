# 菜单系统设计

## 按键定义

| 按键 | 引脚 |
|---|---|
| Key1 | PA17 |
| Key2 | PC5 |

## 按键逻辑

- **短按**：释放时处理（按下的周期数 < HOLD_CNT）
- **长按**：按住超过 HOLD_CNT 次采样周期（8 × 200ms = 1.6s）
- **消抖**：中断层 50ms 消抖

## 菜单结构（二级）

```
┌── 主菜单 ──────────────────────────────────┐
│                                            │
│  > TRACK          ← Key2 切换箭头          │
│    THRESHOLD      ← Key1 进入选中功能       │
│                                            │
│  K1:enter  K2:switch                       │
└────────────────────────────────────────────┘
        │ Key1                     │ Key1
        ▼                         ▼
┌── TRACK ───────────────┐  ┌── THRESHOLD ───────────┐
│                        │  │  子状态1: 选择传感器    │
│  40 cm/s               │  │  v v v                 │
│  L:18.2  R:17.8        │  │  1 2 3 4 5 6 7 8      │
│  >> GO <<              │  │       ^                │
│                        │  │  THR:1000              │
│  K1:+5  K2:-5          │  │                        │
│  K1long1.5s:run        │  │  K1:next              │
│  K2long:back           │  │  K1long:select        │
│                        │  │  K2long:back           │
│                        │  └────────┬───────────────┘
│                        │           │ K1long
│                        │           ▼
│                        │  ┌── 子状态2: 调整阈值 ──┐
│                        │  │ SENSOR 3              │
│                        │  │ THR: 1050             │
│                        │  │                       │
│                        │  │ K1:+50  K2:-50        │
│                        │  │ K1long:OK → 回子状态1 │
│                        │  └───────────────────────┘
└────────────────────────┘
```

## 数据

- `g_gray_threshold[8]` — 每个传感器独立阈值，默认全 1000
- `g_track_speed_cm_s` — 循迹目标速度 cm/s，默认 40
- `run_flag` — 启停标志

## 文件

- `BSP/MENU/menu.h` — 状态枚举、全局变量声明
- `BSP/MENU/menu.c` — 完整状态机 + LCD 绘制 + 按键处理
- `BSP/GRAY/gray.c` — 使用 `g_gray_threshold[0..7]` 替代硬编码 1000
- `BSP/KEY/key.c` — 移除中断中的 `run_flag = 1`（菜单控制启停）
- `User/main.c` — `menu_init()` + `menu_update()` + `run_flag` 控制巡线启停
- `Project/car_sum.uvprojx` — 需添加 `menu.c` 和 MENU 路径
