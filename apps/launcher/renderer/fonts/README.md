# Maple Mono CN Italic

来源：Maple Mono 官方 v7.9 的 [MapleMono-CN.zip](https://github.com/subframe7536/maple-font/releases/download/v7.9/MapleMono-CN.zip)

| 本地文件 | 官方原文件 | 字重 |
| --- | --- | --- |
| `maple-mono-cn-italic.woff2` | `MapleMono-CN-Italic.ttf` | 400 |
| `maple-mono-cn-medium-italic.woff2` | `MapleMono-CN-MediumItalic.ttf` | 500 |
| `maple-mono-cn-semibold-italic.woff2` | `MapleMono-CN-SemiBoldItalic.ttf` | 600 |

使用已有 fontTools 4.29.1 与 Brotli 1.0.9 将完整 TTF 转换为 WOFF2；不裁剪字符、不修改字形、不修改 GSUB 连字规则，保留真实 Italic 字形

三套字体均保留全部 23,398 个字形和 22,732 个字符映射；转换后 GSUB 与原文件逐字节一致，保留官方 `calt` 连字；官方原文件未提供独立 `liga` 特性

许可为 SIL Open Font License 1.1；[OFL.txt](OFL.txt) 原样保留官方压缩包中的 `LICENSE.txt`
