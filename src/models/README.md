# x500_mid360 模型包

```
x500_mid360
 ├─ x500
 │   └─ x500_base
 └─ gemini2
```

| 模型 | 作用 |
|---|---|
| `x500_mid360` | 顶层模型，合并 x500 + gemini2，并添加 MID-360 雷达、雷达 IMU、前视相机 |
| `x500` | 合并 x500_base，添加 4 个电机插件 |
| `x500_base` | 机体本体，含机身网格和飞控 IMU/气压计/磁罗盘/GPS |
| `gemini2` | 下视 RGB 相机 |

所有 include 均为 `merge='true'`，加载后只生成一个 `x500_mid360` 模型。
