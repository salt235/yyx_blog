#!/usr/bin/env node
import { execFile } from 'node:child_process'
import { mkdir, readFile, writeFile, rename, open, unlink, appendFile } from 'node:fs/promises'
import { existsSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { resolve } from 'node:path'
import { parseArgs, promisify } from 'node:util'
import { unwrap } from './data.mjs'
import { decodeFit, normalizeFit } from './fit-data.mjs'
import { detailBatchPlan, syncDetailBatch } from './detail-batch.mjs'

const root = fileURLToPath(new URL('../../', import.meta.url))
const cache = resolve(root, '.local/coros')
const output = resolve(root, 'docs/.vuepress/public/running-data/details')
const statePath = resolve(root, '.github/coros-fit-state.json')
const execute = promisify(execFile)
const { values } = parseArgs({ options: {
  limit: { type: 'string', default: '20' }, issuer: { type: 'string', default: 'https://mcpcn.coros.com' },
  refresh: { type: 'boolean', default: false }, 'dry-run': { type: 'boolean', default: false },
} })

async function download(activity, reserveRequest) {
  const path = resolve(cache, `fit/${activity.id}.fit`)
  if (existsSync(path)) return readFile(path)
  await reserveRequest()
  let result
  try {
    const { stdout } = await execute(process.execPath, [resolve(root, 'node_modules/coros-mcp/dist/cli.js'),
      '--issuer', values.issuer, '--cache-root', cache, '--client-name', 'yyx Running Journal',
      'call-tool', '--tool', 'queryActivityFitFileDownloadUrls', '--arguments-json', JSON.stringify({ labelId: activity.id, sportType: activity.sportType })],
    { cwd: root, timeout: 90000, maxBuffer: 1024 * 1024 })
    const response = JSON.parse(stdout)
    const text = JSON.stringify(response)
    if (/quota|rate.?limit|50 files|24.hour|配额|限流|额度/i.test(text)) throw Object.assign(new Error('COROS FIT 配额不足。'), { code: 'FIT_QUOTA' })
    result = unwrap(response)
  } catch (error) {
    if (error.code === 'FIT_QUOTA') throw error
    throw Object.assign(new Error('COROS FIT 链接请求失败，请检查网络和本机授权。'), { code: 'FIT_TRANSPORT' })
  }
  const urls = typeof result === 'string' ? result.match(/https:\/\/\S+/g) : null
  if (urls?.length !== 1) throw new Error('COROS 未返回唯一 FIT 下载链接，未生成公开详情。')
  // Never print signed URLs or upstream error bodies.
  let buffer
  try {
    const response = await fetch(urls[0], { signal: AbortSignal.timeout(60000) })
    if (!response.ok) throw new Error()
    if (Number(response.headers.get('content-length')) > 32 * 1024 * 1024) throw new Error()
    buffer = Buffer.from(await response.arrayBuffer())
    if (buffer.length > 32 * 1024 * 1024) throw new Error()
  } catch { throw Object.assign(new Error('FIT 下载失败，未生成公开详情。'), { code: 'FIT_TRANSPORT' }) }
  decodeFit(buffer)
  await writeFile(`${path}.tmp`, buffer, { mode: 0o600 })
  await rename(`${path}.tmp`, path)
  return buffer
}

async function main() {
  const limit = Number(values.limit)
  if (!Number.isInteger(limit) || limit < 1 || limit > 20) throw new Error('--limit 必须为 1 至 20。')
  const data = JSON.parse(await readFile(resolve(root, 'docs/.vuepress/public/running-data/activities.json'), 'utf8'))
  const options = { activities: data.activities, output, statePath, limit, refresh: values.refresh }
  if (values['dry-run']) {
    const { batch, completedIds } = await detailBatchPlan(options)
    const available = data.activities.filter(activity => completedIds.has(activity.id)).length
    console.log(`已补全 ${available}/${data.activities.length} 次；本批计划 ${batch.length} 次（不请求 COROS、不写文件）。`)
    for (const activity of batch) console.log(`${activity.date} ${activity.distanceKm} km · ${activity.id}`)
    return
  }
  await mkdir(resolve(cache, 'fit'), { recursive: true, mode: 0o700 })
  const lockPath = resolve(cache, 'sync.lock')
  let lock
  try { lock = await open(lockPath, 'wx', 0o600) } catch { throw new Error('已有 COROS 同步正在运行，请稍后重试。') }
  try {
    const report = await syncDetailBatch({ ...options,
      loadDetail: async (activity, reserveRequest) => normalizeFit(decodeFit(await download(activity, reserveRequest)), activity),
    })
    const message = `本批完成 ${report.completed} 次，失败 ${report.failed} 次；累计 ${report.available}/${report.total} 次，剩余 ${report.remaining} 次；本批请求 FIT ${report.requests} 份。`
    console.log(message)
    if (process.env.GITHUB_STEP_SUMMARY) await appendFile(process.env.GITHUB_STEP_SUMMARY, `### 跑步完整详情补全\n\n${message}\n${report.paused ? '\n额度不足，等待后续任务继续。\n' : ''}`)
    if (report.failed || report.paused) {
      if (process.env.GITHUB_ACTIONS) console.warn('::warning::部分完整详情尚未补全，已保存进度，后续任务将继续。')
      process.exitCode = 1
    }
  } finally { await lock.close(); await unlink(lockPath) }
}

main().catch(error => {
  // Only these local validation errors are safe to display; never print raw HTTP/CLI errors.
  const message = /^[\u4e00-\u9fff]/.test(error.message) ? error.message : '请检查本地文件、网络或授权。'
  console.error(`详情导入未完成：${message} 已保留本地 FIT 缓存。`)
  process.exitCode = 1
})
