# SN 序列号注册（白名单）复刻技术文档

## 0. 目标行为（必须一致）
- ✅ **SN 在白名单** → 直接通过，无需 IMEI，不卡住。
- ❌ **SN 不在白名单** → 显示“联系 林肯飞行家注册”，并阻止下一步。
- ✅ **不依赖 comma.ai 后端**（不再走 pilotauth 注册接口）。

---

## 1. 设计要点
1) **先做序列号白名单判断**，避免 IMEI 缺失导致的注册卡住。  
2) 未授权时：  
   - 直接写入 `DongleId = UnregisteredDevice`  
   - 触发 `Offroad_UnregisteredHardware` 提示（带序列号）  
   - **阻止 onroad/下一步**（恢复 hardwared 启动条件）  
3) 已授权时：  
   - 直接把 `DongleId` 写为序列号（或 `SN:<serial>`）  
   - 不再调用后端注册接口。

---

## 2. 白名单存放位置（推荐）
**仓库内置（仅此一处）：**  
`system/athena/serial_whitelist.txt`

**格式：** 一行一个序列号，例如：
```
1cdcd13e
536beb8b
```

说明：  
- 只读取仓库内置白名单，设备本地 `/persist` 不再参与。  
- 这样可避免 SSH 用户通过 `/persist` 绕过授权。  
- 新增授权必须设备更新到最新仓库版本。

---

## 3. 改动清单
### 3.1 注册逻辑（核心）
文件：`system/athena/registration.py`

**目标：** 在 `register()` 中优先检查 SN 白名单，不再等待 IMEI/调用后端。

**建议新增逻辑（伪代码）：**
```
serial = HARDWARE.get_serial()
whitelist = read_serial_whitelist()

if serial not in whitelist:
  params.put("DongleId", UNREGISTERED_DONGLE_ID)
  set_offroad_alert("Offroad_UnregisteredHardware", True, extra_text=serial)
  return UNREGISTERED_DONGLE_ID

# serial in whitelist
params.put("DongleId", serial)   # 或 "SN:"+serial
set_offroad_alert("Offroad_UnregisteredHardware", False)
return serial
```

**实现建议：**
- 新增函数读取白名单文件（不存在时返回空集合）。
- 放在 IMEI / api_get 注册逻辑之前。
- 这样**不会出现“IMEI: (None, None)”卡住界面**。

---

### 3.2 未授权提示文案
文件：`selfdrive/selfdrived/alerts_offroad.json`

修改 `Offroad_UnregisteredHardware.text` 为：
```
设备未授权（SN: %1）。请联系“林肯飞行家注册”。
```

说明：`%1` 会被 `extra_text`（serial）替换。

---

### 3.3 阻止下一步（强制未注册无法进入 onroad）
文件：`system/hardware/hardwared.py`

恢复/开启注册条件（原本被注释）：
```
startup_conditions["registered_device"] = PC or (params.get("DongleId") != UNREGISTERED_DONGLE_ID)
```

效果：未授权序列号将无法进入 onroad，卡在 offroad 提示界面。

---

## 4. 建议实现代码片段（可直接复制）
### 4.1 `system/athena/registration.py` 新增白名单读取函数
```
def _read_serial_whitelist() -> set[str]:
  repo_path = Path(BASEDIR) / "system" / "athena" / "serial_whitelist.txt"

  def _read(path: Path) -> set[str]:
    if not path.is_file():
      return set()
    serials = set()
    with open(path) as f:
      for line in f:
        s = line.strip()
        if s:
          serials.add(s)
    return serials

  return _read(repo_path)
```

### 4.2 在 `register()` 开头插入（在 IMEI 逻辑前）
```
serial = HARDWARE.get_serial()
whitelist = _read_serial_whitelist()

if serial not in whitelist:
  params.put("DongleId", UNREGISTERED_DONGLE_ID)
  set_offroad_alert("Offroad_UnregisteredHardware", True, extra_text=serial)
  return UNREGISTERED_DONGLE_ID

params.put("DongleId", serial)
set_offroad_alert("Offroad_UnregisteredHardware", False)
return serial
```

> 如果你想保留原后端注册逻辑作为 fallback，则把 `return serial` 改成继续下走原流程即可。

---

## 5. 验证步骤
### 5.1 已授权序列号
1) 在 `/persist/comma/serial_whitelist.txt` 中加入当前设备 SN  
2) 重启设备  
3) 结果：不会卡在 “registering device…IMEI”，可正常进入下一步

### 5.2 未授权序列号
1) 删除或不包含当前 SN  
2) 重启设备  
3) 结果：  
   - 提示：“设备未授权（SN: xxxx）。请联系‘林肯飞行家注册’。”  
   - 不能进入 onroad（下一步被阻止）

---

## 6. 回滚方法
- 恢复 `system/athena/registration.py` 原逻辑  
- 恢复 `alerts_offroad.json` 原文案  
- 注释/移除 `hardwared.py` 中的 `registered_device` 启动条件

---

## 7. 备注
如需支持“批量授权 + 远程同步白名单”，可把白名单文件改成云端拉取并缓存到 `/persist/comma/serial_whitelist.txt`。本方案保持离线可用、无后端依赖。
