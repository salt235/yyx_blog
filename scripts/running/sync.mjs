#!/usr/bin/env node
import { spawnSync, execFile } from 'node:child_process'
import { readFile, writeFile, mkdir, rename, open, unlink } from 'node:fs/promises'
import { existsSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { resolve, dirname } from 'node:path'
import { parseArgs, promisify } from 'node:util'
import { makeDataset, parseRecords, parseDetail, parseLaps, unwrap } from './data.mjs'

const root = fileURLToPath(new URL('../../', import.meta.url))
const cacheRoot = resolve(root, '.local/coros')
const output = resolve(root, 'docs/.vuepress/public/running-data/activities.json')
const cli = resolve(root, 'node_modules/coros-mcp/dist/cli.js')
const execute = promisify(execFile)
const { values, positionals } = parseArgs({ allowPositionals: true, options: {
  from: { type: 'string' }, to: { type: 'string' }, import: { type: 'string' },
  issuer: { type: 'string', default: 'https://mcp.coros.com' },
  'refresh-details': { type: 'boolean', default: false }, help: { type: 'boolean' },
} })

function official(args, interactive = false) {
  if (!existsSync(cli)) throw new Error('请先运行 pnpm install，安装官方 coros-mcp CLI。')
  const result = spawnSync(process.execPath, [cli, '--issuer', values.issuer, '--cache-root', cacheRoot,
    '--client-name', 'yyx Running Journal', ...args], {
    cwd: root, encoding: 'utf8', stdio: interactive ? 'inherit' : 'pipe', timeout: 360000, maxBuffer: 24 * 1024 * 1024,
  })
  // Do not forward upstream response bodies: auth failures may contain tokens.
  if (result.error || result.status !== 0) throw new Error('COROS 调用失败。请检查网络；授权过期时运行 pnpm running:login。')
  return interactive ? null : JSON.parse(result.stdout)
}

async function call(name, args) {
  for (let attempt = 0; attempt < 3; attempt++) {
    try {
      const { stdout } = await execute(process.execPath, [cli, '--issuer', values.issuer, '--cache-root', cacheRoot,
        '--client-name', 'yyx Running Journal', 'call-tool', '--tool', name, '--arguments-json', JSON.stringify(args)], {
        cwd: root, timeout: 90000, maxBuffer: 24 * 1024 * 1024,
      })
      const result = JSON.parse(stdout)
      const value = unwrap(result)
      if (typeof value !== 'string' || !/temporarily unavailable|please try again later/i.test(value)) return result
    } catch {
      // Keep raw subprocess output out of logs: it can contain auth responses.
    }
    if (attempt === 2) throw new Error('COROS 调用连续失败，已保留断点和原有数据。请检查网络或重新授权。')
    await new Promise(resolve => setTimeout(resolve, (attempt + 1) * 1000))
  }
}

function date(value) {
  const d = new Date(`${value}T00:00:00Z`)
  if (!/^\d{4}-\d{2}-\d{2}$/.test(value) || Number.isNaN(+d) || d.toISOString().slice(0, 10) !== value) throw new Error('日期请使用 YYYY-MM-DD。')
  return d
}
const iso = d => d.toISOString().slice(0, 10)
const compact = d => iso(d).replaceAll('-', '')

async function query(start, end, toolName) {
  const limit = 100
  const records = parseRecords(await call(toolName, {
    startDate: compact(start), endDate: compact(end), sportTypeCodes: [100, 101, 102, 103], limit,
  }))
  // Split saturated ranges rather than silently truncating history.
  if (records.length >= limit) {
    if (+start === +end) throw new Error('单日记录达到查询上限，未覆盖原有数据。')
    const middle = new Date(+start + Math.floor((end - start) / 86400000 / 2) * 86400000)
    return [...await query(start, middle, toolName), ...await query(new Date(+middle + 86400000), end, toolName)]
  }
  return records
}

async function main() {
  if (values.help) {
    console.log('pnpm running:login\npnpm running:sync [--from YYYY-MM-DD] [--to YYYY-MM-DD] [--refresh-details]\n仅导出运动摘要、详情和分圈；不导出坐标、token 或原始 MCP 响应。')
    return
  }
  const command = positionals[0] ?? 'sync'
  if (command === 'login' || command === 'login-start' || command === 'login-finish') {
    official([command], true)
    return
  }
  if (command !== 'sync') throw new Error(`未知命令：${command}`)
  await mkdir(cacheRoot, { recursive: true, mode: 0o700 })
  const lock = resolve(cacheRoot, 'sync.lock')
  let handle
  try { handle = await open(lock, 'wx', 0o600) } catch { throw new Error('已有同步正在执行；如果上次被强制中断，请确认进程退出后删除 .local/coros/sync.lock。') }
  try {
    let dataset
    if (values.import) {
      // For agent-assisted imports through an already-authorized official connector.
      const imported = JSON.parse(await readFile(resolve(values.import), 'utf8'))
      const records = parseRecords(imported.records)
      const activities = records.map(activity => ({
        ...activity,
        ...(imported.details?.[activity.id] ? parseDetail(imported.details[activity.id]) : {}),
        splits: imported.laps?.[activity.id] ? parseLaps(imported.laps[activity.id], activity) : [],
        detailsSynced: !!(imported.details?.[activity.id] && imported.laps?.[activity.id]),
      }))
      dataset = makeDataset(activities, imported.coverage)
    } else {
      let previous
      try { previous = JSON.parse(await readFile(output, 'utf8')) } catch (error) { if (error.code !== 'ENOENT') throw error }
      const today = new Intl.DateTimeFormat('en-CA', { timeZone: 'Asia/Shanghai', year: 'numeric', month: '2-digit', day: '2-digit' }).format(new Date())
      const end = date(values.to ?? today)
      const previousEnd = previous?.coverage?.end
      const start = date(values.from ?? (previousEnd ? iso(new Date(+date(previousEnd) - 7 * 86400000)) : iso(new Date(+end - 89 * 86400000))))
      if (start > end) throw new Error('开始日期不能晚于结束日期。')
      const catalog = official(['list-tools', '--refresh'])
      const tool = name => {
        const found = catalog.find(item => item.name === name || item.name.endsWith(`_${name}`))
        if (!found) throw new Error(`官方 MCP 没有返回 ${name}，请检查工具目录。`)
        return found.name
      }
      const recordsTool = tool('querySportRecords'), detailTool = tool('getActivityDetail'), lapsTool = tool('queryActivityLapData')
      const checkpointPath = resolve(cacheRoot, 'pending-sync.json')
      let checkpoint
      try { checkpoint = JSON.parse(await readFile(checkpointPath, 'utf8')) } catch (error) { if (error.code !== 'ENOENT') throw error }
      const resume = checkpoint?.start === iso(start) && checkpoint?.end === iso(end)
      const records = resume ? checkpoint.records : []
      const completed = resume ? checkpoint.completed : {}
      if (resume) console.log(`恢复断点：${Object.keys(completed).length}/${records.length} 次跑步`)
      else {
        for (let cursor = start; cursor <= end;) {
          const rangeEnd = new Date(Math.min(+end, +cursor + 29 * 86400000))
          console.log(`读取 ${iso(cursor)} — ${iso(rangeEnd)}`)
          records.push(...await query(cursor, rangeEnd, recordsTool))
          cursor = new Date(+rangeEnd + 86400000)
        }
      }
      const saveCheckpoint = async () => {
        await writeFile(`${checkpointPath}.tmp`, JSON.stringify({ start: iso(start), end: iso(end), records, completed }), { mode: 0o600 })
        await rename(`${checkpointPath}.tmp`, checkpointPath)
      }
      await saveCheckpoint()
      const old = new Map(previous?.activities?.map(activity => [activity.id, activity]) ?? [])
      const activities = []
      const enrich = async activity => {
        if (completed[activity.id]) return completed[activity.id]
        const cached = old.get(activity.id)
        const unchanged = cached && cached.distanceKm === activity.distanceKm && cached.durationSeconds === activity.durationSeconds
          && cached.heartRate === activity.heartRate && cached.paceSeconds === activity.paceSeconds
        if (unchanged && cached.detailsSynced && !values['refresh-details']) {
          return { ...cached, ...activity }
        }
        const args = { labelId: activity.id, sportType: activity.sportType }
        const detail = parseDetail(await call(detailTool, args))
        const splits = parseLaps(await call(lapsTool, args), activity)
        return { ...activity, ...detail, splits, detailsSynced: true }
      }
      for (let index = 0; index < records.length; index += 3) {
        const batch = records.slice(index, index + 3)
        const results = await Promise.allSettled(batch.map(enrich))
        for (let offset = 0; offset < results.length; offset++) {
          const result = results[offset]
          if (result.status === 'fulfilled') {
            completed[batch[offset].id] = result.value
            activities.push(result.value)
          }
        }
        await saveCheckpoint()
        const failed = results.findIndex(result => result.status === 'rejected')
        if (failed !== -1) throw new Error(`${batch[failed].date} 的详情读取失败：${results[failed].reason.message}`)
        console.log(`详情已完成 ${Math.min(index + 3, records.length)}/${records.length}`)
      }
      // Preserve history outside the queried window; reconcile deletions inside it.
      const kept = (previous?.activities ?? []).filter(activity => activity.date < iso(start) || activity.date > iso(end))
      let coverage = { start: iso(start), end: iso(end) }
      if (previous?.coverage) {
        if (iso(start) > iso(new Date(+date(previous.coverage.end) + 86400000)) || iso(end) < iso(new Date(+date(previous.coverage.start) - 86400000))) {
          throw new Error('新旧同步范围不连续，请补齐中间日期后重试。')
        }
        coverage = { start: [coverage.start, previous.coverage.start].sort()[0], end: [coverage.end, previous.coverage.end].sort().at(-1) }
      }
      dataset = makeDataset([...kept, ...activities], coverage)
    }
    await mkdir(dirname(output), { recursive: true })
    const temporary = `${output}.tmp`
    await writeFile(temporary, `${JSON.stringify(dataset)}\n`)
    await rename(temporary, output)
    await unlink(resolve(cacheRoot, 'pending-sync.json')).catch(error => { if (error.code !== 'ENOENT') throw error })
    console.log(`已同步 ${dataset.activities.length} 次跑步 · ${dataset.coverage.start} — ${dataset.coverage.end}`)
  } finally {
    await handle.close()
    await unlink(lock)
  }
}

main().catch(error => { console.error(error.message); process.exitCode = 1 })
