# 跑步页面与 COROS 同步

页面位于 `/running/`，复用博客的 Vue / VuePress，不需要数据库或常驻 API 服务。

```text
COROS 官方 MCP → 同步脚本 → activities.json → 构建、发布 → 跑步页面
```

## 本地查看

```bash
pnpm docs:preview
```

访问终端显示的本地地址下的 `/running/`。预览读取已构建的产物，修改后先运行 `pnpm docs:build`。开发时可使用 `pnpm docs:dev`；若当前环境遇到文件监听数量限制（EMFILE），使用 `CHOKIDAR_USEPOLLING=1 CHOKIDAR_INTERVAL=1000 pnpm docs:dev`。

历史查询从 2023-01-01 开始。当前 COROS 返回的最早记录为 2024-03-01，2023 年查询未返回跑步记录；查询范围不等于每年都有数据。页面能按月份、类型和日期筛选，查看单次数据和每公里分段。

## 授权与更新

```bash
pnpm running:login
```

在浏览器打开命令输出的 COROS 登录链接，完成授权。脚本不接收账号密码。授权由官方 `coros-mcp@0.1.1` CLI 处理；本地 token 保存在被 Git 忽略的 `.local/coros/` 中，CLI 负责过期刷新。当前聊天的 COROS 连接与本机脚本是独立的授权。

```bash
# 重查上次同步结束前 7 天至今天，更新现有记录并追加新记录
pnpm running:sync

# 导入更早的历史；新范围需要与已同步范围连续
pnpm running:sync --from 2023-01-01 --to 2026-11-07

# 对查询范围内的已有活动也重新获取详情和分圈
pnpm running:sync --from 2026-11-01 --refresh-details
```

如网关的地区识别有问题，授权与同步使用相同地区：

```bash
pnpm running:login --issuer https://mcpcn.coros.com
pnpm running:sync --issuer https://mcpcn.coros.com
```

不要同时运行两个同步任务。脚本会对并发执行加锁；查询或解析失败时保留原有 JSON。运动列表按 30 天分段，达到条数上限后自动缩小日期窗口。详情以最多 3 次活动并行获取，并将进度保存在 `.local/coros/pending-sync.json`；失败后以相同起止日期重跑即可续传，全部成功后才替换公共 JSON。已有记录以字符串 ID 去重，避免 64 位 ID 丢失精度；查询窗口内删除的活动会从展示中移除。默认复用未变化活动的详情，校正过分圈时使用 `--refresh-details`。

## GitHub Actions 每日同步

`.github/workflows/docs.yml` 配置为每天北京时间 **03:00** 执行，cron 为 `0 19 * * *`（UTC 19:00）。实际启动时间可能因 GitHub 调度负载延迟。只在默认的 `main` 分支执行。

流程为：恢复授权 → 增量同步 → 保存数据与刷新后的授权 → 构建博客 → 发布到 `gh-pages`。普通 push 只执行测试、构建和发布，不依赖 COROS 网络或授权。

### 一次性启用

1. 本机已完成 `pnpm running:login` 后，运行 `node scripts/running/auth-state.mjs init`。
2. 将 `.local/coros/github-state-key.txt` 的内容填入仓库 **Settings → Secrets and variables → Actions → New repository secret**，名称为 `COROS_STATE_KEY`。不要把它加入 Git 或发送到聊天中。
3. 将本次页面代码、工作流、公共 JSON 和 `.github/coros-auth.enc` 提交至 `main`。
4. 在仓库 Actions 的 `docs` 工作流中手动点击 **Run workflow** 测试一次。日期留空即增量同步，也可填写起止日期补拉历史。

本仓库已保留原有 `TARGET_DOMAIN` Secret 的使用方式。Actions 需要 `contents: write` 权限以保存数据和发布。提交数据使用 `GITHUB_TOKEN`，不会递归触发另一次 push 工作流。

### 凭证与失败处理

- `.github/coros-auth.enc` 只存 AES-256-GCM 加密后的授权，解密密钥只在本地忽略目录和 GitHub Actions Secret 中。密文不进入网站的公共目录。
- refresh token 轮换后，工作流会把新授权重新加密并保存到仓库；内容未变化时不重新生成密文，减少历史体积。
- 即使活动读取失败，也会尽量保存已刷新的授权；活动 JSON 只有在完整同步成功后才提交。失败时不发布半份数据，旧网站继续可用。
- 同步与发布串行运行，不取消前一个任务，避免并发刷新覆盖授权。
- 若 COROS 撤销授权，需要本机重新登录、重新执行 `auth-state.mjs init`，再提交更新的密文；已有密钥会复用。

## 存储方式

公共数据文件位于 `docs/.vuepress/public/running-data/activities.json`，包含摘要、可用详情和公里分段，跟随源码存储于 `main`，随网站发布到 `gh-pages`。按活动 ID 更新，重复同步不会重复追加相同跑步。

不下载或提交 FIT、GPX、逐秒 GPS/心率序列；这些原始文件远大于当前摘要 JSON。每天提交的是数据变化，Git 会复用未变化的对象，并对历史进行压缩。这里的“全部历史”指所有可查询的跑步记录及当前展示字段，不是所有传感器原始采样。

2026-11-07 完整补拉后：518 次跑步、5,332 个分段，公共 JSON 为 748,442 字节（约 0.75 MB），gzip 为 109,855 字节（约 0.11 MB）；加密授权文件约 1.8 KB。2024 / 2025 / 2026 年分别为 168 / 174 / 176 次，2023 年查询未返回记录。

## 数据与限制

- 公共文件只有经过白名单整理的跑步摘要、详情和分圈，不包含 OAuth token、起点坐标、原始文件链接或健康/睡眠数据。日期、心率和地点/训练备注会展示在页面上。
- 列表距离可能经过 COROS 四舍五入；分圈是更精确的距离，累计值在容差内应一致。分圈距离按厘米换算；配速由分段时间和实际距离计算。运动时间使用 COROS 的 Duration，而不是起止时间戳差。
- 部分活动的总距离与原始分段累计距离不同（例如跑步机距离校准）。保留 COROS 返回的两组原值，页面明确标注差异，不缩放或重算原始分段。
- 只选 1 km 分组，不把 5 km 分组和总计组重复累加。没有公里分组时明确显示缺失。COROS 有时省略极短的末段；保留原始公里分段，不伪造缺失尾段。
- 第一版没有轨迹地图和秒级曲线。后续可通过官方 FIT 下载工具补充，当前不触发 FIT 下载配额。
- 同步脚本只读取数据，不会修改 COROS 的运动、训练计划或日程。
- 公共页面刷新只读 JSON，不会调用 COROS；每日任务启用需要将工作流发布到 `main` 并配置上述 Secret。
- 新数据随下一次博客构建发布。`pnpm docs:build` 不会自动请求 COROS，授权和网络故障不会阻塞普通博客构建。

## 验证

```bash
pnpm running:test
pnpm docs:build
```

官方参考：[MCP 接入](https://support.coros.com/hc/en-us/articles/53181619102996-Build-on-COROS-MCP)、[工具文档](https://github.com/coroslab/COROS-MCP/blob/main/docs/en/tools.md)、[官方 CLI](https://www.npmjs.com/package/coros-mcp)。
