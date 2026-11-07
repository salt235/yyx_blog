import { existsSync } from 'node:fs'
import { mkdir, readFile, writeFile, rename } from 'node:fs/promises'
import { dirname, resolve } from 'node:path'

const day = 86400000
// Leave ten of COROS's shared 50 requests available for other clients.
const requestBudget = 40
const validId = id => typeof id === 'string' && /^\d+$/.test(id)

export async function readJson(path, fallback) {
  try { return JSON.parse(await readFile(path, 'utf8')) }
  catch (error) { if (error.code === 'ENOENT') return fallback; throw error }
}

async function saveJson(path, data) {
  await mkdir(dirname(path), { recursive: true })
  await writeFile(`${path}.tmp`, `${JSON.stringify(data)}\n`)
  await rename(`${path}.tmp`, path)
}

export function selectDetailBatch(activities, completedIds, failures, limit, refresh = false) {
  return activities.filter(activity => refresh || !completedIds.has(activity.id)).sort((a, b) => {
    if (!refresh) {
      const failedA = failures[a.id], failedB = failures[b.id]
      if (!!failedA !== !!failedB) return failedA ? 1 : -1
      if (failedA && failedB && failedA !== failedB) return failedA - failedB
    }
    return b.date.localeCompare(a.date) || b.id.localeCompare(a.id)
  }).slice(0, limit)
}

export async function detailBatchPlan({ activities, output, statePath, limit, refresh = false }) {
  if (!Number.isInteger(limit) || limit < 1 || limit > 20) throw new Error('--limit 必须为 1 至 20。')
  if (activities.some(a => !validId(a.id))) throw new Error('活动 ID 格式无效。')
  const index = await readJson(resolve(output, 'index.json'), { version: 1, ids: [] })
  if (index.version !== 1 || !Array.isArray(index.ids) || index.ids.some(id => !validId(id))) throw new Error('完整详情索引格式无效。')
  const state = await readJson(statePath, { version: 1, requests: [], failures: {} })
  if (state.version !== 1 || !Array.isArray(state.requests) || state.requests.some(time => !Number.isFinite(time))
    || !state.failures || typeof state.failures !== 'object' || Array.isArray(state.failures)
    || Object.entries(state.failures).some(([id, time]) => !validId(id) || !Number.isFinite(time))) throw new Error('补全进度格式无效。')
  const completedIds = new Set(index.ids.filter(id => existsSync(resolve(output, `${id}.json`))))
  const batch = selectDetailBatch(activities, completedIds, state.failures, limit, refresh)
  return { state, completedIds, batch }
}

export async function syncDetailBatch({ activities, output, statePath, limit = 20, refresh = false, loadDetail,
  log = console.log, now = Date.now }) {
  const { state, completedIds, batch } = await detailBatchPlan({ activities, output, statePath, limit, refresh })
  const report = { completed: 0, failed: 0, requests: 0, paused: false, total: activities.length, available: 0, remaining: 0 }
  const pruneRequests = () => { state.requests = state.requests.filter(time => time > now() - day) }
  pruneRequests()
  // Save before calling MCP: a failed response may still have consumed quota.
  const reserveRequest = async () => {
    pruneRequests()
    if (state.requests.length >= requestBudget) throw Object.assign(new Error('滚动 24 小时内已达到本站 40 份请求预算，等待后续任务继续。'), { code: 'FIT_QUOTA' })
    state.requests.push(now())
    await saveJson(statePath, state)
    report.requests++
  }
  for (const activity of batch) {
    log(`补全 ${activity.date} ${activity.distanceKm} km · ${activity.id}`)
    let detail
    try {
      // Recover a file written just before an interrupted manifest update.
      if (!refresh && existsSync(resolve(output, `${activity.id}.json`))) {
        detail = await readJson(resolve(output, `${activity.id}.json`), null)
      }
      if (!detail || detail.version !== 1 || detail.id !== activity.id || !Array.isArray(detail.series) || !Array.isArray(detail.route?.segments)) {
        detail = await loadDetail(activity, reserveRequest)
      }
      if (detail.version !== 1 || detail.id !== activity.id || !Array.isArray(detail.series) || !Array.isArray(detail.route?.segments)) throw new Error('详情数据格式无效。')
    } catch (error) {
      if (error.code === 'FIT_QUOTA') {
        report.paused = true
        log('请求额度不足，已保留本批进度，下次继续。')
        break
      }
      state.failures[activity.id] = now()
      await saveJson(statePath, state)
      report.failed++
      log(`本次未完成 ${activity.id}，后续任务会重试。`)
      if (error.code === 'FIT_TRANSPORT') break
      continue
    }
    await saveJson(resolve(output, `${activity.id}.json`), detail)
    completedIds.add(activity.id)
    await saveJson(resolve(output, 'index.json'), { version: 1, ids: [...completedIds] })
    delete state.failures[activity.id]
    await saveJson(statePath, state)
    report.completed++
  }
  report.available = activities.filter(activity => completedIds.has(activity.id)).length
  report.remaining = report.total - report.available
  return report
}
