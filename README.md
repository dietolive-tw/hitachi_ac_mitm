# Hitachi AC MITM

原生 ESP-IDF + esp-matter 韌體，將日立冷氣遙控器與主機之間「並聯」一顆 ESP32-C6，達成 Matter 智慧家庭整合，**不改裝冷氣主機**。

> 本專案從 esp-matter SDK 拉出成獨立 repo。Build 時仍需要完整 esp-matter SDK 透過 `setup.sh` 自動準備。

## 專案資訊

| 項目 | 值 |
|------|-----|
| 目標晶片 | ESP32-C6 (ESP32-C6-DevKitC-1) @ 160MHz |
| Flash | 16MB |
| ESP-IDF | v5.5.4+ |
| esp-matter | 透過 `setup.sh` clone（espressif/esp-matter main） |
| 廠商名稱 (VendorName) | `Ken` |
| 產品名稱 (ProductName) | `DIY_Matter` |
| 無線模式 | WiFi + Thread 雙模（硬體 PTI 共存） |
| 序列埠 | `/dev/ttyACM0` (USB Serial/JTAG, 115200) |

## 設計理念：非侵入式並聯（MITM）

不改裝冷氣主機，而是在原裝遙控器與冷氣 IR 接收器之間「並聯」一顆 ESP32-C6：

```
┌─────────────┐   IR   ┌──────────────┐   IR LED   ┌──────────┐
│ 原裝遙控器  │ ──→──→ │  ESP32-C6    │ ──→──────→ │ 冷氣主機 │
│ (Hitachi)   │        │ (並聯監聽)   │            │ IR 接收器 │
└─────────────┘        │  VS1838B     │            └──────────┘
                       │  GPIO4 ← RX  │
                       │  GPIO5 → TX  │
                       └──────────────┘
```

- **RX** 監聽遙控器發出的 IR 命令，解碼後同步到 Matter attribute（遙控器 → Apple Home）
- **TX** 在 controller 下達命令時發射 IR（Apple Home → 冷氣）
- **GPIO monitor** 監控冷氣電源/壓縮機狀態，與 Matter attribute 雙向同步

詳細硬體接線、IR 協議、endpoint 配置請見 `main/` 內原始碼註解。

## Quickstart

```bash
# 1. clone this repo
git clone https://github.com/dietolive-tw/hitachi_ac_mitm.git
cd hitachi_ac_mitm

# 2. setup（自動 clone esp-matter + apply SDK patches + symlink example）
./setup.sh

# 3. source ESP-IDF + esp-matter env（每個 shell 一次）
. ~/works/esp32/esp-idf/export.sh
. ~/works/esp32/dev/esp-matter/export.sh

# 4. build / flash / monitor
cd ~/works/esp32/dev/esp-matter/examples/hitachi_ac_mitm
idf.py -DIDF_TARGET=esp32c6 build
idf.py -p /dev/ttyACM0 flash monitor
```

## 為什麼需要 patches/

esp-matter SDK 的 data-model-provider 為 codegen cluster 提供 `FindClusterOnEndpoint()` wrapper，但只有 `flow_measurement` 與 `relative_humidity_measurement` 兩個 cluster 有實作。`temperature_measurement` 與 `fan_control` 漏掉 → 應用程式呼叫 header 宣告的 `FindClusterOnEndpoint` 會 link error。

`patches/` 內是補上這兩個 cluster 的 wrapper（照 `flow_measurement` 既有 pattern）。已發 PR 給 upstream：

| Patch | Upstream PR |
|-------|-------------|
| `esp-matter-temperature-measurement-find-cluster.patch` | espressif/esp-matter#1816 |
| `esp-matter-fan-control-find-cluster.patch` | espressif/esp-matter#1817 |

upstream merge 後可刪除對應 patch 檔，`setup.sh` 會自動跳過已 apply 的 patch。

## OTA 版號覆寫

```bash
idf.py -DIDF_TARGET=esp32c6 -DPROJECT_VER_NUMBER=2 -DPROJECT_VER=2.0 build
```

驗證廠商/產品字串已嵌入 ELF：

```bash
strings -n 3 build/hitachi_ac_mitm.elf | grep -xE "Ken"
strings build/hitachi_ac_mitm.elf | grep -xE "DIY_Matter"
strings build/hitachi_ac_mitm.elf | grep -xE "TEST_VENDOR|TEST_PRODUCT"  # should be empty
```

## 目標硬體

- ESP32-C6-DevKitC-1 @ 160 MHz, 16 MB flash
- 序列埠：`/dev/ttyACM0` (USB Serial/JTAG, 115200)
- WiFi + Thread 雙模（硬體 PTI 共存）

## 授權

請見 `LICENSE`（如未來新增）。
