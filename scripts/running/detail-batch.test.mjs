import test from 'node:test'
import assert from 'node:assert/strict'
import { mkdtemp, readFile, writeFile, mkdir, rm } from 'node:fs/promises'
import { tmpdir } from 'node:os'
import { resolve } from 'node:path'
import { detailBatchPlan, selectDetailBatch, syncDetailBatch } from './detail-batch.mjs'

const activities = ['30', '20', '10'].map((id, i) => ({ id, date: `2026-11-0${3 - i}`, distanceKm: 5 }))
const detail = id => ({ version: 1, id, route: { status: 'missing', segments: [] }, series: [], metrics: {} })
async function fixture(t) {
  const root = await mkdtemp(resolve(tmpdir(), 'yyx-detail-batch-'))
  t.after(() => rm(root, { recursive: true, force: true }))
  return { activities, output: resolve(root, 'details'), statePath: resolve(root, 'state.json'), log: () => {} }
}
const read = async path => JSON.parse(await readFile(path, 'utf8'))

test('按新旧顺序补缺，跳过已完成记录；失败项放到未尝试记录之后', () => {
  assert.deepEqual(selectDetailBatch(activities, new Set(['30']), {}, 20).map(a => a.id), ['20', '10'])
  assert.deepEqual(selectDetailBatch(activities, new Set(), { '30': 1 }, 2).map(a => a.id), ['20', '10'])
  assert.deepEqual(selectDetailBatch(activities, new Set(['30']), { '30': 1 }, 2, true).map(a => a.id), ['30', '20'])
})

test('每条成功立即保存，第二批接着补；全部完成后不再请求 FIT', async t => {
  const options = await fixture(t)
  const requested = []
  const loadDetail = async (activity, reserve) => { await reserve(); requested.push(activity.id); return detail(activity.id) }
  const first = await syncDetailBatch({ ...options, limit: 2, loadDetail })
  assert.equal(first.available, 2)
  assert.equal(first.remaining, 1)
  const second = await syncDetailBatch({ ...options, limit: 2, loadDetail })
  assert.equal(second.completed, 1)
  assert.equal(second.available, 3)
  const third = await syncDetailBatch({ ...options, limit: 20, loadDetail })
  assert.equal(third.requests, 0)
  assert.equal(third.remaining, 0)
  assert.deepEqual(requested, ['30', '20', '10'])
  assert.deepEqual((await read(resolve(options.output, 'index.json'))).ids, ['30', '20', '10'])
})

test('解析失败保留本批成功数据，后续先补其他记录再重试失败项', async t => {
  const options = await fixture(t)
  const first = await syncDetailBatch({ ...options, limit: 2, now: () => 1000,
    loadDetail: async (activity, reserve) => { await reserve(); if (activity.id === '30') throw new Error('bad FIT'); return detail(activity.id) },
  })
  assert.equal(first.failed, 1)
  assert.equal(first.completed, 1)
  assert.deepEqual((await read(resolve(options.output, 'index.json'))).ids, ['20'])
  const next = await detailBatchPlan({ ...options, limit: 20 })
  assert.deepEqual(next.batch.map(a => a.id), ['10', '30'])
  assert.equal(next.state.failures['30'], 1000)
  const report = await syncDetailBatch({ ...options, loadDetail: async activity => detail(activity.id) })
  assert.equal(report.remaining, 0)
  assert.deepEqual((await read(options.statePath)).failures, {})
})

test('请求前记账，滚动 24 小时最多使用本站 40 份预算；过期后恢复', async t => {
  const options = await fixture(t)
  const time = Date.parse('2026-11-07T15:00:00Z')
  await writeFile(options.statePath, JSON.stringify({ version: 1, requests: Array(40).fill(time - 1000), failures: {} }))
  let downloads = 0
  const loadDetail = async (activity, reserve) => { await reserve(); downloads++; return detail(activity.id) }
  const paused = await syncDetailBatch({ ...options, now: () => time, loadDetail })
  assert.equal(paused.paused, true)
  assert.equal(downloads, 0)
  assert.equal(paused.failed, 0)
  const resumed = await syncDetailBatch({ ...options, now: () => time + 86400001, loadDetail })
  assert.equal(resumed.completed, 3)
  assert.equal((await read(options.statePath)).requests.length, 3)
})

test('恢复已落盘但尚未登记的详情不占下载额度；索引文件缺失时重新补齐', async t => {
  const options = await fixture(t)
  await mkdir(options.output)
  await writeFile(resolve(options.output, '30.json'), JSON.stringify(detail('30')))
  await writeFile(resolve(options.output, 'index.json'), JSON.stringify({ version: 1, ids: ['20'] }))
  let downloads = 0
  const report = await syncDetailBatch({ ...options,
    loadDetail: async (activity, reserve) => { await reserve(); downloads++; return detail(activity.id) },
  })
  assert.equal(report.completed, 3)
  assert.equal(downloads, 2)
  assert.equal(report.remaining, 0)
})

test('网络/授权故障停止本批，前面的成功进度和请求额度不丢失', async t => {
  const options = await fixture(t)
  const calls = []
  const report = await syncDetailBatch({ ...options,
    loadDetail: async (activity, reserve) => {
      await reserve(); calls.push(activity.id)
      if (activity.id === '20') throw Object.assign(new Error('network'), { code: 'FIT_TRANSPORT' })
      return detail(activity.id)
    },
  })
  assert.deepEqual(calls, ['30', '20'])
  assert.equal(report.completed, 1)
  assert.equal(report.failed, 1)
  assert.equal((await read(options.statePath)).requests.length, 2)
  assert.deepEqual((await read(resolve(options.output, 'index.json'))).ids, ['30'])
})
