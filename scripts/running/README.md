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

历史查询从 2023-01-01 开始。当前 COROS 返回的最早记录为 2024-03-01，2023 年查询未返回跑步记录；查询范围不等于每年都有数据。页面能点击月度柱形和日历日期筛选，查看单次数据和每公里分段。

## 完整详情与历史补全

```bash
# 查看下一批计划，不调用 COROS 或写入文件
pnpm running:sync-details --limit 20 --dry-run

# 从尚未补全的记录中，每次最多补 20 条
pnpm running:sync-details --limit 20

# 重新整理最近 10 条已有记录，优先复用本地 FIT 缓存
pnpm running:sync-details --limit 10 --refresh
pnpm docs:build
```

脚本通过官方 MCP 的 `queryActivityFitFileDownloadUrls` 获取 FIT，由 Garmin 官方 `@garmin/fitsdk` 校验和解析。原始 FIT 只保存在被忽略的 `.local/coros/fit/`，不会提交到 GitHub。默认按日期从新到旧，跳过已经完成的详情，优先补新训练，再逐步补齐历史。每次最多处理 20 条；全部补齐后，后续只处理新增记录。

每完成一条就保存对应 JSON 和索引，因此中途失败不丢失已完成的进度。重跑跳过已完成记录；本地也会复用已有 FIT。解析失败的记录记入 `.github/coros-fit-state.json`，先让其他未尝试记录继续，再重试失败项，避免某条旧数据一直堵住补全。网络或授权错误会提前停止本批。该状态文件只记录请求时间戳和失败活动 ID，不含授权或原始响应，不发布到网站。

COROS 的 FIT 下载和链接工具共享每平台用户滚动 24 小时 50 份的配额，单次 MCP 调用最多 10 份。本脚本逐条取链接，并在请求前持久化记账；本站自限滚动 24 小时 40 份，为其他客户端预留 10 份。默认每天 20 条，手动重跑也复用同一本账；其他客户端的请求由 COROS 服务端计入实际共享配额，服务端提示额度不足时暂停并保留进度。

公共详情在 `docs/.vuepress/public/running-data/details/<活动 ID>.json`，目录中的 `index.json` 只列出已导入 ID。页面按所选训练加载单个详情，主页面只画无底图路线轮廓；打开“查看完整详情”才加载 Leaflet 地图和 ECharts 曲线。地图使用标准 WGS84 坐标和 OpenStreetMap 底图，底图请求会向 `tile.openstreetmap.org` 发送所查看的地图瓦片区域；保持可见署名，不预下载地图。底图不可用时保留路线并提示。

展示规则：

- 按用户要求完整保留起终点与附近路线；公开 JSON 包含真实路线位置。抽样保留每段首尾点，GPS 缺口分段展示，不跨缺口连线。
- 室内跑明确标注“室内跑 · 无 GPS 轨迹”；户外没有可展示的 GPS 轨迹时也明确说明。未导入的旧训练提示完整详情仍在分批补全，不会误写成没有 GPS。
- 配速由 FIT 的实际速度（m/s）换算为秒/km；速度为零时配速为空。FIT 跑步步频按周期/分钟计数，乘 2 显示双脚步数；坐标从 semicircles 转为度。海拔零值保留，缺失指标不补零。
- 曲线展示经过时间，缺失采样超过 15 秒处断开，采样约束在约 1,400 点并保留缺失边界；原始峰值在抽样前计算。图中可切换心率、海拔、步频和功率，未记录的指标不显示选项。
- 更多指标包含可用的步幅、功率、训练负荷、最高心率/步频、下降、触地时间、垂直振幅/步幅比。保留 COROS 摘要值，不用 FIT 重新覆盖已有摘要和公里分段。

初始试用已导入最近 10 次训练。所有 518 次历史摘要仍然保留，完整路线、曲线和额外指标由后续每日任务分批补齐；2023 年没有返回跑步记录，因此不会生成该年的空活动。

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

流程为：恢复授权 → 增量同步摘要 → 补全最多 20 次完整详情 → 保存数据、进度和刷新后的授权 → 构建博客 → 发布到 `gh-pages`。普通 push 只执行测试、构建和发布，不调用 COROS，不占用 FIT 配额。沿用已配置的 `COROS_STATE_KEY`，无需新增 Secret。

GitHub Actions 的运行摘要会显示本批完成/失败数、累计完成数、剩余数和 FIT 请求数。完整详情补全失败不会阻止发布已成功同步的摘要；失败步骤会给出 warning，当前批次已完成的详情和请求账本仍会提交，后续任务继续。

### 一次性启用

1. 本机已完成 `pnpm running:login` 后，运行 `node scripts/running/auth-state.mjs init`。
2. 将 `.local/coros/github-state-key.txt` 的内容填入仓库 **Settings → Secrets and variables → Actions → New repository secret**，名称为 `COROS_STATE_KEY`。不要把它加入 Git 或发送到聊天中。
3. 将本次页面代码、工作流、公共 JSON 和 `.github/coros-auth.enc` 提交至 `main`。
4. 在仓库 Actions 的 `docs` 工作流中手动点击 **Run workflow** 测试一次。日期留空即增量同步，也可填写起止日期补拉历史。

本仓库已保留原有 `TARGET_DOMAIN` Secret 的使用方式。Actions 需要 `contents: write` 权限以保存数据和发布。提交数据使用 `GITHUB_TOKEN`，不会递归触发另一次 push 工作流。

### 凭证与失败处理

- `.github/coros-auth.enc` 只存 AES-256-GCM 加密后的授权，解密密钥只在本地忽略目录和 GitHub Actions Secret 中。密文不进入网站的公共目录。
- refresh token 轮换后，工作流会把新授权重新加密并保存到仓库；内容未变化时不重新生成密文，减少历史体积。
- 即使活动读取失败，也会尽量保存已刷新的授权。摘要 JSON 只有在摘要同步成功后才提交；详情按活动逐条完成，成功条目与补全账本可以保留，失败条目不登记为完成。
- 同步与发布串行运行，不取消前一个任务，避免并发刷新覆盖授权。
- 若 COROS 撤销授权，需要本机重新登录、重新执行 `auth-state.mjs init`，再提交更新的密文；已有密钥会复用。

## 存储方式

公共数据文件位于 `docs/.vuepress/public/running-data/activities.json`，包含摘要、可用详情和公里分段，跟随源码存储于 `main`，随网站发布到 `gh-pages`。按活动 ID 更新，重复同步不会重复追加相同跑步。

摘要同步不下载 FIT、GPX 或逐秒序列；其后的完整详情任务每次最多下载 20 份 FIT 到忽略目录，将整理、抽样后的路线及曲线 JSON 放入公共目录。云端 runner 上的原始 FIT 随临时环境清理，仓库只保留已整理的详情和进度。每天提交的是数据变化，Git 会复用未变化的对象，并对历史进行压缩。

2026-11-07 完整补拉后：518 次跑步、5,332 个分段，公共 JSON 为 748,442 字节（约 0.75 MB），gzip 为 109,855 字节（约 0.11 MB）；加密授权文件约 1.8 KB。2024 / 2025 / 2026 年分别为 168 / 174 / 176 次，2023 年查询未返回记录。

## 数据与限制

- 公共文件包含经过白名单整理的跑步摘要、分圈及已导入活动的路线和曲线，不包含 OAuth token、原始文件链接或健康/睡眠数据。日期、心率、起终点、完整路线及地点会展示在页面上。
- 列表距离可能经过 COROS 四舍五入；分圈是更精确的距离，累计值在容差内应一致。分圈距离按厘米换算；配速由分段时间和实际距离计算。运动时间使用 COROS 的 Duration，而不是起止时间戳差。
- 部分活动的总距离与原始分段累计距离不同（例如跑步机距离校准）。保留 COROS 返回的两组原值，页面明确标注差异，不缩放或重算原始分段。
- 只选 1 km 分组，不把 5 km 分组和总计组重复累加。没有公里分组时明确显示缺失。COROS 有时省略极短的末段；保留原始公里分段，不伪造缺失尾段。
- 完整详情每天分批更新，未补全记录保留摘要；每公里分段放在完整详情弹窗内。全部补齐后，每日任务自动只补新增训练。
- 同步脚本只读取数据，不会修改 COROS 的运动、训练计划或日程。
- 公共页面刷新只读 JSON，不会调用 COROS；每日任务启用需要将工作流发布到 `main` 并配置上述 Secret。
- 新数据随下一次博客构建发布。`pnpm docs:build` 不会自动请求 COROS，授权和网络故障不会阻塞普通博客构建。

## 验证

```bash
pnpm running:test
pnpm docs:build
```

官方参考：[MCP 接入](https://support.coros.com/hc/en-us/articles/53181619102996-Build-on-COROS-MCP)、[工具文档](https://github.com/coroslab/COROS-MCP/blob/main/docs/en/tools.md)、[官方 CLI](https://www.npmjs.com/package/coros-mcp)。
