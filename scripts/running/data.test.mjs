import test from 'node:test'
import assert from 'node:assert/strict'
import { makeDataset, parseRecords, parseDetail, parseLaps, seconds } from './data.mjs'

const text = `Sport Records — 2026-11-01 to 2026-11-07 (2 records)
========================

1. Outdoor Run — 2026-11-04
   Location: 南京市
   Start Coordinates: 31.000000, 118.000000
   Time Window: startTimestamp=1793791608 | endTimestamp=1793794163
   Duration: 42:35 | Distance: 8.01 km
   Average Pace: 5:19 /km | Avg HR: 145 bpm | Calories: 525 kcal
   LabelId: 481518802146721993 | SportType: 100

2. Outdoor Run — 2026-11-01
   Duration: 5:21 | Distance: 949 m
   Average Pace: 5:38 /km
   LabelId: 481518802146721992 | SportType: 100`
const wrap = content => ({ content: [{ type: 'text', text: typeof content === 'string' ? JSON.stringify(content) : JSON.stringify(content) }] })

test('保留 64 位 ID，换算米，使用运动用时而非时间戳差，并去除坐标', () => {
  const activities = parseRecords(wrap(text))
  assert.equal(activities.length, 2)
  assert.equal(activities[0].id, '481518802146721993')
  assert.equal(activities[0].durationSeconds, 2555)
  assert.equal(activities[0].paceSeconds, 319)
  assert.equal(activities[1].distanceKm, 0.949)
  assert.equal(activities[1].heartRate, null)
  assert.ok(!JSON.stringify(activities).includes('31.000000'))
  assert.ok(!JSON.stringify(activities).includes('startTimestamp'))
})

test('空列表可接受；工具错误、截断列表、上游格式变化不能变成空数据', () => {
  assert.deepEqual(parseRecords(wrap('Sport Records — 2026-11-07 to 2026-11-07 (0 records)')), [])
  assert.deepEqual(parseRecords(wrap('No sport records found from 2026-11-07 to 2026-11-07.')), [])
  assert.throws(() => parseRecords({ isError: true }))
  assert.throws(() => parseRecords(wrap(text.replace('(2 records)', '(3 records)'))))
  assert.throws(() => parseRecords(wrap('Access denied')))
})

test('详情保留缺失值；零爬升与未知爬升有区别', () => {
  const detail = parseDetail(wrap('Outdoor Run Activity Details\nAverage Cadence: 176 spm\nElevation Gain / Loss: 0 m / 0 m\nAverage Power: 65535 W'))
  assert.equal(detail.cadence, 176)
  assert.equal(detail.elevationGainMeters, 0)
  assert.equal(detail.powerWatts, null)
  assert.equal(detail.strideMeters, null)
})

test('分圈使用厘米和秒，只选公里组，保留不足一公里的尾段', () => {
  const group = { type: 2, lapDistance: 100000, laps: [
    { distance: 100000, time: 300, avgHr: 150, avgCadence: 180 },
    { distance: 20000, time: 66, avgHr: 155 },
  ] }
  const laps = parseLaps(wrap({ lapGroups: [{ type: -1, laps: [] }, group, { type: 11, lapDistance: 500000, laps: [] }] }), { distanceKm: 1.2, durationSeconds: 366 })
  assert.equal(laps.length, 2)
  assert.equal(laps[0].distanceKm, 1)
  assert.equal(laps[1].paceSeconds, 330)
  assert.equal(laps[1].cadence, null)
  assert.throws(() => parseLaps(wrap({ lapGroups: [group] }), { distanceKm: 1200 }))
  assert.throws(() => parseLaps(wrap({ lapGroups: [group] }), { distanceKm: 1.2, durationSeconds: 36600 }))
  assert.deepEqual(parseLaps(wrap({ lapGroups: [] }), { distanceKm: 1 }), [])
})

test('同步以字符串 ID 去重，最新记录优先；时长跨小时和进位正确', () => {
  const records = parseRecords(wrap(text))
  const dataset = makeDataset([...records, records[0]], { start: '2026-11-01', end: '2026-11-07' })
  assert.equal(dataset.activities.length, 2)
  assert.equal(dataset.activities[0].date, '2026-11-04')
  assert.equal(seconds('3:25:19'), 12319)
  assert.throws(() => seconds('3:75:19'))
})

test('COROS 省略极短末段时保留原始公里组，不伪造尾段', () => {
  const laps = [227.25, 240.13, 243.87, 238.42, 231.25].map(time => ({ distance: 100000, time }))
  const parsed = parseLaps(wrap({ lapGroups: [{ type: 2, lapDistance: 100000, laps }] }), { distanceKm: 5.01, durationSeconds: 1184 })
  assert.equal(parsed.length, 5)
  assert.equal(parsed.reduce((sum, lap) => sum + lap.distanceKm, 0), 5)
})

test('校准后的总距离与原始分圈可以不同，保留两者而不重算分圈', () => {
  const group = { type: 2, lapDistance: 100000, laps: [{ distance: 100000, time: 300 }, { distance: 20000, time: 60 }] }
  const total = { type: -1, laps: [{ distance: 120000, time: 360 }] }
  const parsed = parseLaps(wrap({ lapGroups: [group, total] }), { distanceKm: 1.15, durationSeconds: 360 })
  assert.equal(parsed.reduce((sum, lap) => sum + lap.distanceKm, 0), 1.2)
  assert.throws(() => parseLaps(wrap({ lapGroups: [group, total] }), { distanceKm: 115, durationSeconds: 360 }))
  assert.equal(parseDetail(wrap('Indoor Run Activity Details\nTotal Ascent: 125 m')).elevationGainMeters, 125)
})

test('间歇训练的训练步和零距离休息不能误认为公里分段', () => {
  const steps = { type: 2, lapDistance: 100000, laps: [{ distance: 120000, time: 360 }, { distance: 0, time: 30 }] }
  const auto = { type: 10, lapDistance: 100000, laps: [{ distance: 100000, time: 300 }, { distance: 20000, time: 90 }] }
  const total = { type: -1, laps: [{ distance: 120000, time: 390 }] }
  const parsed = parseLaps(wrap({ lapGroups: [steps, auto, total] }), { distanceKm: 1.2, durationSeconds: 390 })
  assert.equal(parsed.length, 2)
  assert.equal(parsed[1].distanceKm, 0.2)
  assert.equal(parsed[1].paceSeconds, 450)
})
