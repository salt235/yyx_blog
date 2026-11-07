import test from 'node:test'
import assert from 'node:assert/strict'
import { decodeFit, normalizeFit, routeSegments, thin } from './fit-data.mjs'

const semicircles = degree => degree * 2 ** 31 / 180
const start = new Date('2026-11-04T11:00:00Z')
const activity = { id: '481518802146721993', date: '2026-11-04', sportType: 100, distanceKm: 1 }
const record = (seconds, extra = {}) => ({ timestamp: new Date(+start + seconds * 1000), distance: seconds * 3, speed: 3.125, heartRate: 150, cadence: 88, enhancedAltitude: 0, ...extra })
const messages = records => ({ sessionMesgs: [{ sport: 'running', startTime: start, totalDistance: 1000, totalElapsedTime: 320, totalTimerTime: 310, avgVerticalOscillation: 90, avgStanceTime: 230 }], recordMesgs: records })

test('FIT 采用 semicircles 坐标、m/s 配速、双脚步频；海拔零与缺失值不同', () => {
  const data = normalizeFit(messages([record(0), record(1, { speed: 0, cadence: 0, power: 0, heartRate: undefined }), record(2)]), activity)
  assert.equal(data.id, activity.id)
  assert.equal(data.series[0].pace, 320)
  assert.equal(data.series[0].cadence, 176)
  assert.equal(data.series[0].elevation, 0)
  assert.equal(data.series[1].pace, null)
  assert.equal(data.series[1].heartRate, null)
  assert.equal(data.series[1].cadence, 0)
  assert.equal(data.series[1].power, 0)
  assert.equal(data.metrics.verticalOscillationCm, 9)
  assert.equal(data.metrics.groundContactMs, 230)
  assert.equal(data.metrics.timerSeconds, 310)
  assert.equal(data.metrics.elapsedSeconds, 320)
  assert.equal(data.route.status, 'missing')
  const gps = [118, 118.004, 118.006, 118.01].map((lon, i) => record(i, { positionLat: semicircles(32), positionLong: semicircles(lon) }))
  const route = normalizeFit(messages(gps), activity).route
  assert.equal(route.status, 'available')
  assert.deepEqual(route.segments, [[[118, 32], [118.004, 32], [118.006, 32], [118.01, 32]]])
})

test('室内跑即使 FIT 含坐标也不展示 GPS；跨采样缺口保持断线', () => {
  const coordinates = { positionLat: semicircles(32), positionLong: semicircles(118) }
  const data = normalizeFit(messages([record(0, coordinates), record(30, coordinates)]), { ...activity, sportType: 101 })
  assert.equal(data.route.status, 'indoor')
  assert.deepEqual(data.route.segments, [])
  assert.equal(data.series[1].pace, null)
  assert.equal(data.series[1].heartRate, null)
})

test('完整保留起终点与附近路线；GPS 缺口不连线，抽样保留各段端点', () => {
  const startPoint = [118, 32], endPoint = [118.02, 32]
  const points = [startPoint, [118.001, 32], [118.004, 32], [118.005, 32.001], null, [118.01, 32], startPoint, [118.006, 32], [118.007, 32], [118.019, 32], endPoint]
  const result = routeSegments(points)
  assert.equal(result.length, 2)
  assert.deepEqual(result, [points.slice(0, 4), points.slice(5)])
  const loop = [startPoint, [118.001, 32], [118.001, 32.001], startPoint]
  assert.deepEqual(routeSegments(loop), [loop])
  assert.deepEqual(routeSegments(points, 4), [[startPoint, points[3]], [points[5], endPoint]])
  assert.deepEqual(thin([1, 2, 3, 4, 5], 3), [1, 3, 5])
})

test('坏 FIT、跨日期/错配活动拒绝发布；精确的高峰指标在抽样前计算', () => {
  assert.throws(() => decodeFit(Buffer.from('not FIT')))
  assert.throws(() => normalizeFit(messages([record(0)]), { ...activity, date: '2026-11-05' }), /日期不符/)
  assert.throws(() => normalizeFit(messages([record(0)]), { ...activity, distanceKm: 20 }), /距离不符/)
  const records = Array.from({ length: 3000 }, (_, i) => record(i, { heartRate: i === 17 ? 190 : 140 }))
  const result = normalizeFit(messages(records), activity)
  assert.equal(result.metrics.maxHeartRate, 190)
  assert.ok(result.series.length < 1405)
  assert.equal(result.series.at(-1).seconds, 2999)
})
